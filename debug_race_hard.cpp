// DEBUG RACE -- Round 3
// Lock-free ring buffer for a real-time sensor data pipeline.
// Passes data between a high-frequency producer (1kHz sensor driver)
// and a slower consumer (100Hz perception pipeline).
//
// Something is wrong. Find the bugs.
//
// Compile: g++ -std=c++17 -O2 -o debug3 debug_race_hard.cpp -lpthread
// Note:    -O2 matters — some bugs only appear with optimization.
// Run:     ./debug3

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
    float data[64];
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

public:
    RingBuffer() : head_(0), tail_(0) {
        memset(buffer_, 0, sizeof(buffer_));
    }

    // Producer: write one element
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next_head = (head + 1) & (Capacity - 1);

        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false;  // buffer full
        }

        buffer_[head & (Capacity - 1)] = item;

        head_.store(next_head, std::memory_order_relaxed);

        return true;
    }

    // Consumer: read one element
    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);

        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // buffer empty
        }

        item = buffer_[tail & (Capacity - 1)];

        tail_.store((tail + 1) & (Capacity - 1), std::memory_order_relaxed);

        return true;
    }

    size_t size() const {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t tail = tail_.load(std::memory_order_relaxed);
        return (head - tail) & (Capacity - 1);
    }

    bool empty() const {
        return head_.load(std::memory_order_relaxed) ==
               tail_.load(std::memory_order_relaxed);
    }

private:
    T buffer_[Capacity];
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
};

// --- Test harness ---

static constexpr size_t BUFFER_SIZE = 100;

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
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
}

void consumer() {
    uint64_t expected_seq = 0;
    uint64_t out_of_order = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        SensorPacket pkt;

        if (g_buffer.pop(pkt)) {
            uint32_t expected_checksum = pkt.computeChecksum();
            if (pkt.checksum != expected_checksum) {
                g_corrupted.fetch_add(1, std::memory_order_relaxed);
            }

            if (pkt.sequence < expected_seq && expected_seq > 0) {
                out_of_order++;
            }
            expected_seq = pkt.sequence + 1;

            g_consumed.fetch_add(1, std::memory_order_relaxed);
        }

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

    std::this_thread::sleep_for(std::chrono::seconds(3));
    g_running.store(false, std::memory_order_relaxed);

    prod_thread.join();
    cons_thread.join();

    std::cout << "\n=== Results ===\n";
    std::cout << "Produced:  " << g_produced.load() << "\n";
    std::cout << "Consumed:  " << g_consumed.load() << "\n";
    std::cout << "Dropped:   " << g_dropped.load() << "\n";
    std::cout << "Corrupted: " << g_corrupted.load() << "\n";

    double drop_rate = (g_produced.load() > 0)
        ? 100.0 * g_dropped.load() / g_produced.load()
        : 0.0;

    std::cout << "\nDrop rate: " << drop_rate << "%\n";

    if (g_corrupted.load() > 0) {
        std::cout << "\n*** CORRUPTION DETECTED ***\n";
        std::cout << "This indicates a data race in the ring buffer.\n";
    }

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
