#include <Arduino.h>
#include <SPI.h>

// This standalone sketch validates the accelerator without Wi-Fi or Telegram.
// It intentionally contains no credentials. After hardware validation, call
// fpgaPrepareB() and fpgaRunWithResidentB() from the original application.

namespace Accelerator {

constexpr uint8_t CMD_SET_N        = 0xA0;
constexpr uint8_t CMD_LOAD_A_BLOCK = 0xA1;
constexpr uint8_t CMD_LOAD_B       = 0xB0;
constexpr uint8_t CMD_COMPUTE_TILE = 0xC0;

constexpr int CS_PIN   = 5;
// Software-remapped over the existing wires: GPIO4 drives FPGA pin 42
// (GCLKC_1) as SCLK; FPGA pin 40 returns MISO to GPIO18; the original
// pin32-to-GPIO19 wire carries only the low-speed DONE flag.
constexpr int DONE_PIN = 19;
constexpr int SCK_PIN  = 4;
constexpr int MISO_PIN = 18;
constexpr int MOSI_PIN = 23;

// The 64x64 remapped interface is hardware-validated at 19 MHz. The
// speed 0 command remains available as a conservative 100 kHz fallback.
uint32_t spiFrequencyHz = 19000000;
bool spiBusConfigured = false;

constexpr uint32_t FPGA_TIMEOUT_US = 100000;
constexpr int MAX_N = 64;

SPISettings settings() {
    return SPISettings(spiFrequencyHz, MSBFIRST, SPI_MODE0);
}

bool validSize(int n) {
    return n >= 4 && n <= MAX_N && (n % 4) == 0;
}

void configureSpiBus() {
    if (spiBusConfigured)
        SPI.endTransaction();
    SPI.beginTransaction(settings());
    spiBusConfigured = true;
}

void beginTransaction() {
    digitalWrite(CS_PIN, LOW);
}

void endTransaction() {
    digitalWrite(CS_PIN, HIGH);
}

void sendCommand(uint8_t command, const uint8_t* payload, size_t length) {
    beginTransaction();
    SPI.transfer(command);
    if (length != 0)
        SPI.transferBytes(payload, nullptr, length);
    endTransaction();
}

bool fpgaConfigure(int n) {
    if (!validSize(n))
        return false;

    const uint8_t sizeByte = static_cast<uint8_t>(n);
    sendCommand(CMD_SET_N, &sizeByte, 1);
    return true;
}

bool fpgaPrepareB(const int8_t* matrixB, int n) {
    if (!fpgaConfigure(n))
        return false;

    sendCommand(
        CMD_LOAD_B,
        reinterpret_cast<const uint8_t*>(matrixB),
        static_cast<size_t>(n) * n
    );
    return true;
}

void fpgaLoadTwoRowsOfA(const int8_t* twoRows, int n) {
    sendCommand(
        CMD_LOAD_A_BLOCK,
        reinterpret_cast<const uint8_t*>(twoRows),
        static_cast<size_t>(2) * n
    );
}

bool waitForDone() {
    const uint32_t start = micros();
    while (!digitalRead(DONE_PIN)) {
        if (static_cast<uint32_t>(micros() - start) > FPGA_TIMEOUT_US)
            return false;
    }
    return true;
}

bool fpgaComputeTile(uint8_t tileColumn) {
    sendCommand(CMD_COMPUTE_TILE, &tileColumn, 1);
    return waitForDone();
}

void fpgaReadTile(int32_t tile[8]) {
    uint8_t zeros[32] = {};
    uint8_t bytes[32] = {};

    beginTransaction();
    SPI.transferBytes(zeros, bytes, sizeof(bytes));
    endTransaction();

    for (int i = 0; i < 8; ++i) {
        const uint32_t raw =
            static_cast<uint32_t>(bytes[i * 4 + 0])       |
            (static_cast<uint32_t>(bytes[i * 4 + 1]) << 8)  |
            (static_cast<uint32_t>(bytes[i * 4 + 2]) << 16) |
            (static_cast<uint32_t>(bytes[i * 4 + 3]) << 24);
        tile[i] = static_cast<int32_t>(raw);
    }
}

// B must already be resident in the FPGA. A is streamed two rows at a time;
// each 2x4 C tile is read immediately, so the FPGA never needs a full C buffer.
bool fpgaRunWithResidentB(const int8_t* matrixA, int32_t* matrixC, int n) {
    if (!validSize(n))
        return false;

    for (int rowBase = 0; rowBase < n; rowBase += 2) {
        fpgaLoadTwoRowsOfA(&matrixA[rowBase * n], n);

        for (int colBase = 0; colBase < n; colBase += 4) {
            const uint8_t tileColumn = static_cast<uint8_t>(colBase / 4);
            if (!fpgaComputeTile(tileColumn))
                return false;

            int32_t tile[8];
            fpgaReadTile(tile);

            for (int tileRow = 0; tileRow < 2; ++tileRow)
                for (int tileCol = 0; tileCol < 4; ++tileCol)
                    matrixC[(rowBase + tileRow) * n + colBase + tileCol] =
                        tile[tileRow * 4 + tileCol];
        }
    }
    return true;
}

void cpuReferenceMultiply(
    const int8_t* matrixA,
    const int8_t* matrixB,
    int32_t* matrixC,
    int n
) {
    for (int row = 0; row < n; ++row) {
        for (int col = 0; col < n; ++col) {
            int32_t sum = 0;
            for (int k = 0; k < n; ++k)
                sum += static_cast<int32_t>(matrixA[row * n + k])
                     * static_cast<int32_t>(matrixB[k * n + col]);
            matrixC[row * n + col] = sum;
        }
    }
}

bool compareMatrices(const int32_t* expected, const int32_t* actual, int n) {
    for (int i = 0; i < n * n; ++i) {
        if (expected[i] != actual[i]) {
            Serial.printf(
                "Mismatch at [%d,%d]: CPU=%ld FPGA=%ld\n",
                i / n,
                i % n,
                static_cast<long>(expected[i]),
                static_cast<long>(actual[i])
            );
            Serial.print("First four CPU/FPGA pairs:");
            const int shown = (n * n < 4) ? n * n : 4;
            for (int j = 0; j < shown; ++j) {
                Serial.printf(
                    " %ld/%ld",
                    static_cast<long>(expected[j]),
                    static_cast<long>(actual[j])
                );
            }
            Serial.println();
            return false;
        }
    }
    return true;
}

void fillDeterministicMatrices(int8_t* matrixA, int8_t* matrixB, int n) {
    for (int i = 0; i < n * n; ++i) {
        // Small signed values exercise signed arithmetic without hiding errors
        // behind overflow or saturation.
        matrixA[i] = static_cast<int8_t>(((i * 3 + 1) % 9) - 4);
        matrixB[i] = static_cast<int8_t>(((i * 7 + 2) % 9) - 4);
    }
}

int64_t matrixChecksum(const int32_t* matrix, int n) {
    int64_t checksum = 0;
    for (int i = 0; i < n * n; ++i)
        checksum += static_cast<int64_t>(matrix[i]) * (i + 1);
    return checksum;
}

struct Matrices {
    int8_t* a = nullptr;
    int8_t* b = nullptr;
    int32_t* fpga = nullptr;
    int32_t* cpu = nullptr;

    bool allocate(int n, bool needCpu) {
        const size_t elements = static_cast<size_t>(n) * n;
        a = static_cast<int8_t*>(malloc(elements));
        b = static_cast<int8_t*>(malloc(elements));
        fpga = static_cast<int32_t*>(malloc(elements * sizeof(int32_t)));
        if (needCpu)
            cpu = static_cast<int32_t*>(malloc(elements * sizeof(int32_t)));
        return a && b && fpga && (!needCpu || cpu);
    }

    void release() {
        free(a); free(b); free(fpga); free(cpu);
        a = nullptr; b = nullptr; fpga = nullptr; cpu = nullptr;
    }
};

void runCommand(int n) {
    if (!validSize(n)) {
        Serial.println("Size must be a multiple of 4 from 4 through 64.");
        return;
    }

    Matrices matrices;
    if (!matrices.allocate(n, false)) {
        Serial.println("Allocation failed.");
        matrices.release();
        return;
    }

    fillDeterministicMatrices(matrices.a, matrices.b, n);
    const bool ok = fpgaPrepareB(matrices.b, n)
                 && fpgaRunWithResidentB(matrices.a, matrices.fpga, n);

    if (ok) {
        Serial.printf(
            "FPGA run complete: %dx%d, checksum=%lld\n",
            n,
            n,
            matrixChecksum(matrices.fpga, n)
        );
    } else {
        Serial.println("FPGA timeout or invalid configuration.");
    }
    matrices.release();
}

void testCommand(int n) {
    if (!validSize(n)) {
        Serial.println("Size must be a multiple of 4 from 4 through 64.");
        return;
    }

    Matrices matrices;
    if (!matrices.allocate(n, true)) {
        Serial.println("Allocation failed.");
        matrices.release();
        return;
    }

    fillDeterministicMatrices(matrices.a, matrices.b, n);
    cpuReferenceMultiply(matrices.a, matrices.b, matrices.cpu, n);

    const bool completed = fpgaPrepareB(matrices.b, n)
                        && fpgaRunWithResidentB(matrices.a, matrices.fpga, n);

    if (!completed)
        Serial.println("SELF-TEST FAILED: FPGA timeout.");
    else if (compareMatrices(matrices.cpu, matrices.fpga, n))
        Serial.printf("SELF-TEST PASSED: %dx%d signed INT8.\n", n, n);
    else
        Serial.println("SELF-TEST FAILED: result mismatch.");

    matrices.release();
}

void benchmarkCommand(int n) {
    if (!validSize(n)) {
        Serial.println("Size must be a multiple of 4 from 4 through 64.");
        return;
    }

    Matrices matrices;
    if (!matrices.allocate(n, true)) {
        Serial.println("Allocation failed.");
        matrices.release();
        return;
    }
    fillDeterministicMatrices(matrices.a, matrices.b, n);

    const int repetitions = (n <= 16) ? 50 : ((n <= 32) ? 10 : 3);

    uint32_t start = micros();
    if (!fpgaPrepareB(matrices.b, n)) {
        Serial.println("FPGA configuration failed.");
        matrices.release();
        return;
    }
    const uint32_t weightLoadUs = micros() - start;

    start = micros();
    for (int i = 0; i < repetitions; ++i) {
        if (!fpgaRunWithResidentB(matrices.a, matrices.fpga, n)) {
            Serial.println("FPGA timeout during benchmark.");
            matrices.release();
            return;
        }
    }
    const float fpgaAverageUs =
        static_cast<float>(micros() - start) / repetitions;

    start = micros();
    for (int i = 0; i < repetitions; ++i)
        cpuReferenceMultiply(matrices.a, matrices.b, matrices.cpu, n);
    const float cpuAverageUs =
        static_cast<float>(micros() - start) / repetitions;

    if (!compareMatrices(matrices.cpu, matrices.fpga, n)) {
        Serial.println("Benchmark rejected: result mismatch.");
        matrices.release();
        return;
    }

    Serial.printf("Benchmark %dx%d at %.1f MHz SPI\n", n, n, spiFrequencyHz / 1e6);
    Serial.printf("One-time B load: %.3f ms\n", weightLoadUs / 1000.0f);
    Serial.printf("ESP32 reference: %.3f ms\n", cpuAverageUs / 1000.0f);
    Serial.printf("FPGA resident-B path: %.3f ms\n", fpgaAverageUs / 1000.0f);
    Serial.printf("End-to-end speedup: %.3fx\n", cpuAverageUs / fpgaAverageUs);

    matrices.release();
}

void setSpiFrequency(int mhz) {
    if (mhz == 0) {
        spiFrequencyHz = 100000;
        configureSpiBus();
        Serial.println("SPI frequency reset to validated 100 kHz.");
        return;
    }
    if (mhz < 1 || mhz > 40) {
        Serial.println("Use speed 0 for 100 kHz, or speed M for any integer 1..40 MHz.");
        return;
    }
    spiFrequencyHz = static_cast<uint32_t>(mhz) * 1000000UL;
    configureSpiBus();
    Serial.printf("SPI frequency set to %d MHz. Run self-tests again.\n", mhz);
}

void probeSpiWiring(int frequencyKHz) {
    if (frequencyKHz < 100 || frequencyKHz > 10000) {
        Serial.println("Probe frequency must be 100..10000 kHz.");
        return;
    }

    if (spiBusConfigured) {
        SPI.endTransaction();
        spiBusConfigured = false;
    }
    SPI.beginTransaction(SPISettings(
        static_cast<uint32_t>(frequencyKHz) * 1000UL,
        MSBFIRST,
        SPI_MODE0
    ));
    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(2);
    const int doneBeforeClock = digitalRead(DONE_PIN);
    const uint8_t returned = SPI.transfer(0xA5);
    const int doneAfterClock = digitalRead(DONE_PIN);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    configureSpiBus();
    delayMicroseconds(2);
    const int doneAfterCs = digitalRead(DONE_PIN);

    Serial.printf(
        "PROBE %dkHz echo=0x%02X done=%d/%d/%d expected=0xA5,0/1/0\n",
        frequencyKHz,
        returned,
        doneBeforeClock,
        doneAfterClock,
        doneAfterCs
    );
}

void printHelp() {
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  test N    CPU/FPGA correctness test; N is a multiple of 4, max 64");
    Serial.println("  run N     FPGA-only final-style run; no comparison or timing");
    Serial.println("  bench N   optional measured CPU/FPGA benchmark");
    Serial.println("  speed 0   restore the hardware-validated 100 kHz setting");
    Serial.println("  speed M   set any integer SPI frequency from 1 to 40 MHz");
    Serial.println("  probe K   temporary SPI diagnostic at K kHz (100..10000)");
    Serial.println("Default validated setting: 19 MHz. Run test 4/8/16/32/64 after programming.");
}

}  // namespace Accelerator

void setup() {
    using namespace Accelerator;

    Serial.begin(115200);
    pinMode(CS_PIN, OUTPUT);
    pinMode(DONE_PIN, INPUT);
    digitalWrite(CS_PIN, HIGH);

    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    configureSpiBus();

    delay(500);
    // After a true power-on, the FPGA SPI clock-domain synchronizers may drop
    // the very first short transaction. Two harmless SET_N commands establish
    // parser state before any user-visible run; the second also covers a lost
    // first command without adding overhead to benchmarked matrix execution.
    fpgaConfigure(4);
    fpgaConfigure(4);
    Serial.println("ESP32 + FPGA 4x4-tile / 64x64 matrix accelerator ready.");
    Serial.println("No benchmark runs automatically.");
    printHelp();
}

void loop() {
    using namespace Accelerator;

    if (!Serial.available())
        return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    const int separator = line.indexOf(' ');
    const String command = (separator < 0) ? line : line.substring(0, separator);
    const int value = (separator < 0) ? 0 : line.substring(separator + 1).toInt();

    if (command == "test")
        testCommand(value);
    else if (command == "run")
        runCommand(value);
    else if (command == "bench")
        benchmarkCommand(value);
    else if (command == "speed")
        setSpiFrequency(value);
    else if (command == "probe")
        probeSpiWiring(value);
    else
        printHelp();
}
