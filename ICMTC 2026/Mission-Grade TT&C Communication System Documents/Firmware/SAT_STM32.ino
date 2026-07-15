#include <SPI.h>
#include <LoRa.h>

// -----------------------------
// LoRa Pins
// -----------------------------
#define SS      PA4
#define RST     PB0
#define DIO0    PB1
#define LED     PC13

// -----------------------------
// Addressing
// -----------------------------
#define GCS_ADDR   1
#define SAT_ADDR   2

// -----------------------------
// Commands
// -----------------------------
#define CMD_LED_ON    0x01
#define CMD_LED_OFF   0x02
#define CMD_BLINK     0x03
#define CMD_STATUS    0x10
#define CMD_ACK       0x11
#define CMD_NACK      0x12

// -----------------------------
// Logical frame (9 bytes, protected by CRC+MAC):
// [SYNC][SRC][DST][SEQ][CMD][DATA][MAC][CRC_HI][CRC_LO]
// -----------------------------
#define SYNC_BYTE     0xAA
#define AUTH_KEY      0x5A     // shared lightweight key (demo only) - must match GCS
#define LOGICAL_LEN   9

// -----------------------------
// FEC: Hamming(7,4) per nibble -> 18 codeword bytes, block-interleaved (3x6)
// The satellite stays on a fixed carrier; Doppler compensation is performed
// entirely on the ground segment (both TX pre-correction and RX correction),
// which is the standard architecture for a non-GPS CubeSat radio.
// -----------------------------
#define ENCODED_LEN   18
#define INTLV_ROWS    3
#define INTLV_COLS    6
#define BASE_FREQ_HZ  433000000UL

uint8_t logicalRx[LOGICAL_LEN];
uint8_t logicalTx[LOGICAL_LEN];
uint8_t encodedRx[ENCODED_LEN];
uint8_t encodedTx[ENCODED_LEN];
uint8_t fecBufTx[ENCODED_LEN];
uint8_t fecBufRx[ENCODED_LEN];

bool ledState = false;

// -----------------------------
// Anti-replay sliding window
// -----------------------------
#define REPLAY_WINDOW 8

uint8_t seenSeq[REPLAY_WINDOW];
uint8_t seenIndex = 0;
bool windowPrimed = false;

// -----------------------------
// CRC16-CCITT
// -----------------------------
uint16_t crc16_ccitt(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

// -----------------------------
// Lightweight MAC (XOR keyed checksum)
// -----------------------------
uint8_t computeMAC(uint8_t src, uint8_t dst, uint8_t seq, uint8_t cmd, uint8_t data)
{
    return (src ^ dst ^ seq ^ cmd ^ data ^ AUTH_KEY);
}

// -----------------------------
// Build / validate logical (pre-FEC) frame
// -----------------------------
void buildLogicalFrame(uint8_t *buf, uint8_t src, uint8_t dst, uint8_t seq, uint8_t cmd, uint8_t data)
{
    buf[0] = SYNC_BYTE;
    buf[1] = src;
    buf[2] = dst;
    buf[3] = seq;
    buf[4] = cmd;
    buf[5] = data;
    buf[6] = computeMAC(src, dst, seq, cmd, data);

    uint16_t crc = crc16_ccitt(buf, 7);

    buf[7] = (crc >> 8) & 0xFF;
    buf[8] = crc & 0xFF;
}

bool validateLogicalFrame(uint8_t *buf)
{
    if (buf[0] != SYNC_BYTE)
        return false;

    uint16_t crcCalc = crc16_ccitt(buf, 7);
    uint16_t crcRecv = ((uint16_t)buf[7] << 8) | buf[8];

    if (crcCalc != crcRecv)
        return false;

    uint8_t macCalc = computeMAC(buf[1], buf[2], buf[3], buf[4], buf[5]);

    if (macCalc != buf[6])
        return false;

    return true;
}

// -----------------------------
// Hamming(7,4) single-error-correcting code
// codeword bit layout (bit0..bit6): p0 p1 d0 p2 d1 d2 d3
// -----------------------------
uint8_t hamming74Encode(uint8_t nibble)
{
    uint8_t d0 = (nibble >> 0) & 1;
    uint8_t d1 = (nibble >> 1) & 1;
    uint8_t d2 = (nibble >> 2) & 1;
    uint8_t d3 = (nibble >> 3) & 1;

    uint8_t p0 = d0 ^ d1 ^ d3;
    uint8_t p1 = d0 ^ d2 ^ d3;
    uint8_t p2 = d1 ^ d2 ^ d3;

    uint8_t codeword = 0;
    codeword |= (p0 << 0);
    codeword |= (p1 << 1);
    codeword |= (d0 << 2);
    codeword |= (p2 << 3);
    codeword |= (d1 << 4);
    codeword |= (d2 << 5);
    codeword |= (d3 << 6);

    return codeword;
}

uint8_t hamming74Decode(uint8_t codeword, bool *corrected)
{
    *corrected = false;

    uint8_t p0 = (codeword >> 0) & 1;
    uint8_t p1 = (codeword >> 1) & 1;
    uint8_t d0 = (codeword >> 2) & 1;
    uint8_t p2 = (codeword >> 3) & 1;
    uint8_t d1 = (codeword >> 4) & 1;
    uint8_t d2 = (codeword >> 5) & 1;
    uint8_t d3 = (codeword >> 6) & 1;

    uint8_t s0 = p0 ^ d0 ^ d1 ^ d3;
    uint8_t s1 = p1 ^ d0 ^ d2 ^ d3;
    uint8_t s2 = p2 ^ d1 ^ d2 ^ d3;

    uint8_t syndrome = s0 | (s1 << 1) | (s2 << 2);

    if (syndrome != 0)
    {
        uint8_t errorBit = syndrome - 1;
        codeword ^= (1 << errorBit);
        *corrected = true;

        d0 = (codeword >> 2) & 1;
        d1 = (codeword >> 4) & 1;
        d2 = (codeword >> 5) & 1;
        d3 = (codeword >> 6) & 1;
    }

    return d0 | (d1 << 1) | (d2 << 2) | (d3 << 3);
}

// -----------------------------
// Block interleaver: ROWS x COLS, write row-major, read column-major
// -----------------------------
void interleave(uint8_t *in, uint8_t *out)
{
    for (uint8_t c = 0; c < INTLV_COLS; c++)
        for (uint8_t r = 0; r < INTLV_ROWS; r++)
            out[c * INTLV_ROWS + r] = in[r * INTLV_COLS + c];
}

void deinterleave(uint8_t *in, uint8_t *out)
{
    for (uint8_t c = 0; c < INTLV_COLS; c++)
        for (uint8_t r = 0; r < INTLV_ROWS; r++)
            out[r * INTLV_COLS + c] = in[c * INTLV_ROWS + r];
}

// -----------------------------
// TX / RX pipelines
// -----------------------------
void encodeAndSend(uint8_t *logical)
{
    for (uint8_t i = 0; i < LOGICAL_LEN; i++)
    {
        uint8_t loNibble = logical[i] & 0x0F;
        uint8_t hiNibble = (logical[i] >> 4) & 0x0F;

        fecBufTx[i * 2 + 0] = hamming74Encode(loNibble);
        fecBufTx[i * 2 + 1] = hamming74Encode(hiNibble);
    }

    interleave(fecBufTx, encodedTx);

    LoRa.beginPacket();
    LoRa.write(encodedTx, ENCODED_LEN);
    LoRa.endPacket();
}

int decodeReceived(uint8_t *encoded, uint8_t *logicalOut)
{
    int correctedCount = 0;

    deinterleave(encoded, fecBufRx);

    for (uint8_t i = 0; i < LOGICAL_LEN; i++)
    {
        bool corrLo = false;
        bool corrHi = false;

        uint8_t loNibble = hamming74Decode(fecBufRx[i * 2 + 0], &corrLo);
        uint8_t hiNibble = hamming74Decode(fecBufRx[i * 2 + 1], &corrHi);

        if (corrLo) correctedCount++;
        if (corrHi) correctedCount++;

        logicalOut[i] = (hiNibble << 4) | loNibble;
    }

    return correctedCount;
}

// -----------------------------
// Replay protection helpers
// -----------------------------
bool isReplay(uint8_t seq)
{
    if (!windowPrimed)
        return false;

    for (uint8_t i = 0; i < REPLAY_WINDOW; i++)
    {
        if (seenSeq[i] == seq)
            return true;
    }

    return false;
}

void recordSeq(uint8_t seq)
{
    seenSeq[seenIndex] = seq;
    seenIndex = (seenIndex + 1) % REPLAY_WINDOW;
    windowPrimed = true;
}

// -----------------------------
// Send ACK / NACK
// -----------------------------
void sendResponse(uint8_t dst, uint8_t seq, bool ok)
{
    buildLogicalFrame(logicalTx, SAT_ADDR, dst, seq, ok ? CMD_ACK : CMD_NACK, ledState);
    encodeAndSend(logicalTx);
}

// -----------------------------
// Setup
// -----------------------------
void setup()
{
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);   // Black Pill LED is active LOW -> start OFF

    Serial.begin(115200);

    LoRa.setPins(SS, RST, DIO0);

    if (!LoRa.begin(BASE_FREQ_HZ))
    {
        Serial.println("LoRa Failed");
        while (1);
    }

    Serial.println("Satellite Ready");
}

// -----------------------------
// Loop
// -----------------------------
void loop()
{
    if (LoRa.parsePacket() != ENCODED_LEN)
        return;

    LoRa.readBytes(encodedRx, ENCODED_LEN);

    int corrected = decodeReceived(encodedRx, logicalRx);

    if (!validateLogicalFrame(logicalRx))
    {
        Serial.print("DROPPED: bad CRC/MAC after FEC (corrected=");
        Serial.print(corrected);
        Serial.println(")");
        return;
    }

    uint8_t src  = logicalRx[1];
    uint8_t dst  = logicalRx[2];
    uint8_t seq  = logicalRx[3];
    uint8_t cmd  = logicalRx[4];
    uint8_t data = logicalRx[5];

    if (dst != SAT_ADDR)
        return;

    if (isReplay(seq))
    {
        Serial.print("DROPPED: replay seq=");
        Serial.println(seq);
        return;   // no response sent for replayed frames
    }

    recordSeq(seq);

    if (corrected > 0)
    {
        Serial.print("FEC corrected ");
        Serial.print(corrected);
        Serial.println(" bit error(s)");
    }

    switch (cmd)
    {
        case CMD_LED_ON:
            digitalWrite(LED, LOW);
            ledState = true;
            Serial.println("LED ON");
            sendResponse(src, seq, true);
            break;

        case CMD_LED_OFF:
            digitalWrite(LED, HIGH);
            ledState = false;
            Serial.println("LED OFF");
            sendResponse(src, seq, true);
            break;

        case CMD_BLINK:
            Serial.println("BLINK");

            for (int i = 0; i < data; i++)
            {
                digitalWrite(LED, LOW);
                delay(250);
                digitalWrite(LED, HIGH);
                delay(250);
            }

            digitalWrite(LED, ledState ? LOW : HIGH);
            sendResponse(src, seq, true);
            break;

        case CMD_STATUS:
            Serial.println("STATUS REQUEST");
            sendResponse(src, seq, true);
            break;

        default:
            Serial.println("UNKNOWN COMMAND");
            sendResponse(src, seq, false);
            break;
    }
}
