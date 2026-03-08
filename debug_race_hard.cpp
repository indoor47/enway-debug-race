// DEBUG RACE -- Exercise 3: HARD
// Bug difficulty: Hard
// Domain: Thread-safe ring buffer for real-time sensor data pipeline
//
// INSTRUCTIONS:
// This is a lock-free(ish) ring buffer used to pass sensor data between
// a high-frequency producer (sensor driver, 1kHz) and a slower consumer
// (perception pipeline, 100Hz). It has 4 bugs, some of which only manifest
// under specific timing conditions.
//
// Compile: g++ -std=c++17 -O2 -o debug3 debug_race_hard.cpp -lpthread
// (Note: -O2 is important -- some bugs only appear with optimization)
//
// Run: ./debug3

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cassert>
#include <vector>
#include <cmath>

// --- Sensor data packet ---
struct SensorPacket {
    uint64_t sequence;
    double timestamp;
    float data[64];      // e.g., 64 range readings from a sensor
    uint32_t checksum;

    uint32_t computeChecksum() const {
        uint32_t sum = 0;
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
        for (size_t i = 0; i < sizeof(data); ++i) {
            sum += bytes[i];
        }
        return sum;
    }
};

// --- Ring Buffer ---
// Single-producer, single-consumer lock-free ring buffer
template <typename T, size_t Capacity>
class RingBuffer {
    static_assert(Capacity > 0, "Capacity must be positive");
    // BUG 1: No power-of-two check. The modulo optimization below assumes
    // Capacity is a power of 2. If it's not, the mask-based indexing is wrong.
    // With Capacity=100 (not power of 2), `index & (Capacity - 1)` gives
    // wrong results. Should use `index % Capacity` for arbitrary sizes,
    // or static_assert that Capacity is power of 2.

public:
    RingBuffer() : head_(0), tail_(0) {
        // Zero-initialize the buffer
        memset(buffer_, 0, sizeof(buffer_));
    }

    // Producer: write one element
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next_head = (head + 1) & (Capacity - 1);  // BUG 1: wrong if not power of 2

        // Check if full
        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false;  // buffer full
        }

        buffer_[head & (Capacity - 1)] = item;  // BUG 1 again

        // BUG 2: The store to head_ should use memory_order_release to ensure
        // the buffer write is visible before the head update.
        // Using relaxed means the consumer might read the new head but see
        // stale data in the buffer slot on weakly-ordered architectures (ARM).
        head_.store(next_head, std::memory_order_relaxed);  // BUG: should be release

        return true;
    }

    // Consumer: read one element
    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);

        // Check if empty
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // buffer empty
        }

        item = buffer_[tail & (Capacity - 1)];  // BUG 1 again

        // BUG 3: Missing memory fence. On ARM/weakly-ordered architectures,
        // the read of buffer_[tail] might be reordered AFTER the store to tail_.
        // This means the producer could overwrite the slot before we finish reading.
        // Need acquire on the head load above (which we have) and release here.
        tail_.store((tail + 1) & (Capacity - 1), std::memory_order_relaxed);  // BUG: should be release

        return true;
    }

    size_t size() const {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t tail = tail_.load(std::memory_order_relaxed);
        // BUG 4: This underflows when tail > head (after wrap-around)
        // With mask-based indexing AND non-power-of-two capacity, this is even more broken
        return (head - tail) & (Capacity - 1);  // Can underflow to huge number
    }

    bool empty() const {
        return head_.load(std::memory_order_relaxed) ==
               tail_.load(std::memory_order_relaxed);
    }

private:
    T buffer_[Capacity];
    // BUG: head_ and tail_ on same cache line causes false sharing
    // In a real implementation, you'd pad these to separate cache lines
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
};

// --- Test harness ---

static constexpr size_t BUFFER_SIZE = 100;  // NOT a power of 2! This triggers BUG 1.
// Should be 128 (or use % instead of &)

RingBuffer<SensorPacket, BUFFER_SIZE> g_buffer;

std::atomic<bool> g_running{true};
std::atomic<uint64_t> g_produced{0};
std::atomic<uint64_t> g_consumed{0};
std::atomic<uint64_t> g_dropped{0};
std::atomic<uint64_t> g_corrupted{0};

void producer() {
    uint64_t seq = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        SensorPacket pkt;
        pkt.sequence = seq;
        pkt.timestamp = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        // Fill with recognizable pattern
        for (int i = 0; i < 64; ++i) {
            pkt.data[i] = static_cast<float>(seq * 100 + i);
        }
        pkt.checksum = pkt.computeChecksum();

        if (g_buffer.push(pkt)) {
            g_produced.fetch_add(1, std::memory_order_relaxed);
        } else {
            g_dropped.fetch_add(1, std::memory_order_relaxed);
        }

        seq++;

        // Simulate 1kHz sensor
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
}

void consumer() {
    uint64_t expected_seq = 0;
    uint64_t out_of_order = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        SensorPacket pkt;

        if (g_buffer.pop(pkt)) {
            // Verify data integrity
            uint32_t expected_checksum = pkt.computeChecksum();
            if (pkt.checksum != expected_checksum) {
                g_corrupted.fetch_add(1, std::memory_order_relaxed);
            }

            // Check for sequence gaps (some drops are expected)
            if (pkt.sequence < expected_seq && expected_seq > 0) {
                out_of_order++;
            }
            expected_seq = pkt.sequence + 1;

            g_consumed.fetch_add(1, std::memory_order_relaxed);
        }

        // Simulate 100Hz consumer
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }

    if (out_of_order > 0) {
        std::cout << "WARNING: " << out_of_order << " out-of-order packets detected!\n";
    }
}

int main() {
    std::cout << "Ring buffer test: BUFFER_SIZE=" << BUFFER_SIZE << "\n";
    std::cout << "Producer: 1kHz, Consumer: 100Hz\n";
    std::cout << "Running for 3 seconds...\n\n";

    std::thread prod_thread(producer);
    std::thread cons_thread(consumer);

    // Run for 3 seconds
    std::this_thread::sleep_for(std::chrono::seconds(3));
    g_running.store(false, std::memory_order_relaxed);

    prod_thread.join();
    cons_thread.join();

    std::cout << "\n=== Results ===\n";
    std::cout << "Produced:  " << g_produced.load() << "\n";
    std::cout << "Consumed:  " << g_consumed.load() << "\n";
    std::cout << "Dropped:   " << g_dropped.load() << "\n";
    std::cout << "Corrupted: " << g_corrupted.load() << "\n";

    uint64_t total = g_consumed.load() + g_dropped.load();
    double drop_rate = (g_produced.load() > 0)
        ? 100.0 * g_dropped.load() / g_produced.load()
        : 0.0;

    std::cout << "\nDrop rate: " << drop_rate << "%\n";

    if (g_corrupted.load() > 0) {
        std::cout << "\n*** CORRUPTION DETECTED ***\n";
        std::cout << "This indicates a data race in the ring buffer.\n";
    }

    // Expected: ~3000 produced (1kHz * 3s), ~300 consumed (100Hz * 3s)
    // With buffer size 100, we expect significant drops
    // With correct implementation: ZERO corrupted packets

    // Sanity checks
    bool passed = true;

    if (g_corrupted.load() > 0) {
        std::cout << "FAIL: Data corruption detected\n";
        passed = false;
    }

    if (g_consumed.load() == 0) {
        std::cout << "FAIL: No packets consumed\n";
        passed = false;
    }

    if (passed) {
        std::cout << "\nAll checks passed.\n";
    } else {
        std::cout << "\nSome checks FAILED -- there are bugs in the ring buffer.\n";
    }

    return passed ? 0 : 1;
}

// === ANSWER KEY (for facilitator) ===
//
// BUG 1 (Lines ~50, 63, 70, 78, 87): Non-power-of-two capacity with bitmask indexing
//   BUFFER_SIZE = 100, but `index & (Capacity - 1)` only works when Capacity
//   is a power of 2 (like 64, 128, 256).
//   100 - 1 = 99 = 0b01100011. Using & with this mask:
//     - index=100: 100 & 99 = 4 (not 0!)
//     - index=127: 127 & 99 = 27 (not 27... wait, 127 & 99 = 99 & 127... let's see:
//       99 = 01100011, 127 = 01111111, AND = 01100011 = 99... but should be 27)
//   This means the buffer accesses wrong slots, causing data corruption.
//   FIX: Either change BUFFER_SIZE to 128, or replace all `& (Capacity-1)` with `% Capacity`.
//
// BUG 2 (Line ~70): Memory ordering too weak on head store
//   `head_.store(next_head, std::memory_order_relaxed)` should be `memory_order_release`
//   Without release, on ARM the consumer might see the updated head_ before
//   the buffer write is globally visible, reading stale/partial data.
//   On x86 this is "accidentally correct" due to TSO, which is why it's so insidious.
//   FIX: `head_.store(next_head, std::memory_order_release);`
//
// BUG 3 (Line ~83): Memory ordering too weak on tail store
//   Same issue in reverse. The consumer reads the buffer, then updates tail.
//   Without release on the tail store, the producer might see the updated tail
//   before the consumer's read is complete, and overwrite data being read.
//   FIX: `tail_.store((tail + 1) ..., std::memory_order_release);`
//
// BUG 4 (Line ~87-90): size() underflow
//   When the buffer wraps and tail > head (in terms of raw counter values),
//   `(head - tail)` underflows to a huge number. The & mask makes it worse
//   because the mask is wrong for non-power-of-two.
//   FIX: `return (head - tail + Capacity) % Capacity;`
//   (But note: size() with atomics is inherently racy in SPSC -- it's an approximation)
//
// DIFFICULTY ANALYSIS:
//   - BUG 1 is findable by careful reading but requires understanding bitmask math
//   - BUG 2 and 3 require understanding C++ memory model / std::atomic ordering
//     This is VERY hard for humans but some AI tools know the patterns
//   - BUG 4 is a classic ring buffer size calculation bug
//
// WHY THIS EXERCISE IS VALUABLE:
//   - It's the kind of code C++ robotics engineers actually write
//   - Memory ordering bugs are platform-specific (x86 hides them, ARM exposes them)
//   - It demonstrates that AI tools can reason about concurrent code patterns
//   - But it also shows limits: AI might miss the non-power-of-two interaction
//   - The conversation ABOUT these bugs is more valuable than finding them
//
// EXPECTED AI BEHAVIOR:
//   - Power-of-two issue: ~70% chance AI catches it (it's a well-known pattern)
//   - Memory ordering: ~50% chance AI identifies the specific issue (it knows the
//     patterns but might not reason through the reordering correctly)
//   - size() underflow: ~80% chance AI catches it
//   - Overall: AI will probably find 2-3 of 4 bugs and might suggest some false positives
