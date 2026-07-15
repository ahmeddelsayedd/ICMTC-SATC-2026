#include <SPI.h>
#include <LoRa.h>
#include <math.h>

// -----------------------------
// LoRa Pins
// -----------------------------
#define SS      PA4
#define RST     PB0
#define DIO0    PB1

// -----------------------------
// UART to ESP32 (bridge to PC GUI)
// -----------------------------
HardwareSerial ESP(PA3, PA2);   // RX , TX

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
#define AUTH_KEY      0x5A     // shared lightweight key (demo only) - must match satellite
#define LOGICAL_LEN   9

// -----------------------------
// FEC: Hamming(7,4) per nibble -> 18 codeword bytes (1 byte per 4-bit nibble)
// then block-interleaved (3 rows x 6 cols) to spread burst errors across codewords
// -----------------------------
#define ENCODED_LEN   18
#define INTLV_ROWS    3
#define INTLV_COLS    6

uint8_t logicalTx[LOGICAL_LEN];
uint8_t logicalRx[LOGICAL_LEN];
uint8_t encodedTx[ENCODED_LEN];
uint8_t encodedRx[ENCODED_LEN];
uint8_t fecBufTx[ENCODED_LEN];
uint8_t fecBufRx[ENCODED_LEN];

// -----------------------------
// Reliability parameters
// -----------------------------
#define MAX_RETRIES   3
#define ACK_TIMEOUT   800   // ms

uint8_t mySeq = 0;

// -----------------------------
// Doppler compensation (ground-segment side)
// Simplified cosine model of a LEO pass: max shift at AOS/LOS, zero at TCA.
// Real orbit propagation (SGP4 + TLE) would replace computeDopplerShift()
// in a flight-representative implementation; documented as an assumption
// in the System Design Report.
// -----------------------------
#define BASE_FREQ_HZ        433000000UL
#define MAX_DOPPLER_HZ       3500L      // representative UHF LEO worst-case shift
#define PASS_DURATION_MS     600000UL   // 10 minute simulated pass

unsigned long passStartMillis = 0;
bool passActive = false;
// doppler shift(problem)
long computeDopplerShift()
{
    if (!passActive)
        return 0;

    unsigned long elapsed = millis() - passStartMillis;

    if (elapsed > PASS_DURATION_MS)
    {
        passActive = false;
        return 0;
    }

    float phase = (PI * (float)elapsed) / (float)PASS_DURATION_MS;
    long shift = (long)(MAX_DOPPLER_HZ * cos(phase));

    return shift;
}
// doppler compensation (solution)
void applyDopplerCompensation()
{
    long shift = computeDopplerShift();
    LoRa.setFrequency(BASE_FREQ_HZ + shift);
}

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
// synchronization byte, addressing information, sequence
// number, command, payload data, authentication code (MAC
// -----------------------------
void buildLogicalFrame(uint8_t *buf, uint8_t src, uint8_t dst, uint8_t seq, uint8_t cmd, uint8_t data)
{
   // Store the synchronization byte, addressing fields,
    // sequence number, command, payload, and authentication
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
// checking the synchronization byte, CRC16 checksum,
// and message authentication code (MAC).
bool validateLogicalFrame(uint8_t *buf)
{
    if (buf[0] != SYNC_BYTE)
        return false;
 // Recalculate the CRC and compare it with the received
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

// Decodes a 7-bit codeword, correcting a single bit error if present.
// Sets *corrected = true if a correction was applied.
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
        uint8_t errorBit = syndrome - 1;   // 0-indexed bit position within codeword
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
// Spreads a burst of ROWS consecutive corrupted wire-bytes across ROWS
// distinct Hamming codewords instead of hammering a single codeword.
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
// Full TX pipeline: logical frame ->encode Hamming(7,4) per nibble -> interleave -> LoRa
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

    applyDopplerCompensation();

    LoRa.beginPacket();
    LoRa.write(encodedTx, ENCODED_LEN);
    LoRa.endPacket();
}

// -----------------------------
// Full RX pipeline: LoRa -> deinterleave -> Hamming decode -> logical frame
// Returns number of bit errors corrected by FEC (for KPI logging), or -1 on
// structural failure (should not happen given fixed-length codewords).
// -----------------------------
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
// Send command with retransmission + ACK matching by sequence number
// Reports KPI data (latency, retries, doppler, FEC corrections) to GUI
// -----------------------------
bool sendCommandWithRetry(uint8_t cmd, uint8_t data)
{
    uint8_t seq = mySeq++;

    buildLogicalFrame(logicalTx, GCS_ADDR, SAT_ADDR, seq, cmd, data);
 // Transmission and Retransmission Loop

    for (uint8_t attempt = 0; attempt <= MAX_RETRIES; attempt++)
    {
        unsigned long txTime = millis();
        long dopplerAtTx = computeDopplerShift();

        encodeAndSend(logicalTx);
 // Send transmission details to the Ground Control
        ESP.print("TX seq=");
        ESP.print(seq);
        ESP.print(" attempt=");
        ESP.print(attempt);
        ESP.print(" doppler_hz=");
        ESP.println(dopplerAtTx);
 
        unsigned long start = millis();
// Wait for ACK or NACK
        while (millis() - start < ACK_TIMEOUT)
        {
            applyDopplerCompensation();
 // Verify packet integrity, destination address,
                // packet type, and matching sequence number.
            if (LoRa.parsePacket() == ENCODED_LEN)
            {
                LoRa.readBytes(encodedRx, ENCODED_LEN);

                int corrected = decodeReceived(encodedRx, logicalRx);

                if (!validateLogicalFrame(logicalRx))
                    continue;

                if (logicalRx[2] != GCS_ADDR)
                    continue;

                if (logicalRx[4] != CMD_ACK && logicalRx[4] != CMD_NACK)
                    continue;

                if (logicalRx[3] != seq)
                    continue;   // stale or mismatched ACK, ignore

                unsigned long latency = millis() - txTime;

                if (logicalRx[4] == CMD_NACK)
                {
                    ESP.print("NACK seq=");
                    ESP.print(seq);
                    ESP.print(" corrected=");
                    ESP.println(corrected);
                    return false;
                }
 // Process Positive Acknowledgement
                ESP.print("ACK seq=");
                ESP.print(seq);
                ESP.print(" latency_ms=");
                ESP.print(latency);
                ESP.print(" retries=");
                ESP.print(attempt);
                ESP.print(" corrected=");
                ESP.println(corrected);

                return true;
            }
        }
    }
//-ve 
    ESP.print("TIMEOUT seq=");
    ESP.println(seq);

    return false;
}

// -----------------------------
// Setup
// One-time initialization: bring up the UART link to the ESP32/GUI,
// configure and initialize the LoRa radio on the base UHF frequency
// -----------------------------
void setup()
{
    ESP.begin(115200);

    LoRa.setPins(SS, RST, DIO0);

    if (!LoRa.begin(BASE_FREQ_HZ))
    {
        ESP.println("LoRa Failed");
        while (1);
    }

    ESP.println("GCS READY");
}

// -----------------------------
// Main Loop - parse commands coming from GUI via ESP32
// Recognized commands are sent to the satellite
// -----------------------------
void loop()
{
    if (ESP.available())
    {
        String line = ESP.readStringUntil('\n');
        line.trim();

        if (line == "LED_ON")
        {
            sendCommandWithRetry(CMD_LED_ON, 0);
        }
        else if (line == "LED_OFF")
        {
            sendCommandWithRetry(CMD_LED_OFF, 0);
        }
        else if (line.startsWith("BLINK"))
        {
            uint8_t n = 5;
            int idx = line.indexOf(':');

            if (idx > 0)
                n = line.substring(idx + 1).toInt();

            sendCommandWithRetry(CMD_BLINK, n);
        }
        else if (line == "STATUS")
        {
            sendCommandWithRetry(CMD_STATUS, 0);
        }
        else if (line == "PASS_START")
        {
            passStartMillis = millis();
            passActive = true;
            ESP.println("PASS STARTED");
        }
        else
        {
            ESP.println("UNKNOWN COMMAND");
        }
    }
}