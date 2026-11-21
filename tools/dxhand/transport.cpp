// Keep the upstream ABI and the exclusive USB descriptor in a single owner.
#include "a32_native_linux.cpp"
#include "a3/detail/Protocol.h"

API int dxhand_driver_off(Context* c) noexcept {
    return invoke(c, "driver_off", [&] {
        c->stopSending();
        c->writeAll(a3::packSelfPwr(a3::kBusAll, a3::kNodeAll,
                                    a3::kCmdDisable, a3::kPwrDrvAll));
    });
}

// The first byte commits a packet. Never queue a new target for a blocked
// descriptor. Once started, finish a partial packet with the native writer.
API int dxhand_send_before(Context* c, const float* self, const float* mit,
                           int64_t deadline, SendTiming* timing) noexcept {
    bool expired = false;
    int ok = invoke(c, "dxhand_send_before", [&] {
        if (!timing) throw Failure(6, "missing timing output");
        c->setTargets(self, mit);
        std::unique_lock<std::mutex> lock(c->stateMutex);
        if (!c->connected || c->tcp) throw Failure(1, "connected USB hand required");
        auto packet = a32wire::targets(c->commandSequence++, c->targetSelf, c->targetMit);
        const auto earliest = c->lastSendStart + 1000000;
        lock.unlock();
        std::this_thread::sleep_until(Clock::time_point(std::chrono::nanoseconds(earliest)));
        const auto started = nowNs();
        if (started >= deadline) {
            c->stopSending();
            expired = true;
            return;
        }
        const auto count = ::write(c->fd, packet.data(), packet.size());
        if (count <= 0) throw Failure(7, ioError("USB position write"));
        if (static_cast<size_t>(count) < packet.size())
            c->writeAll(a32wire::Bytes(packet.begin() + count, packet.end()));
        lock.lock();
        const auto finished = nowNs();
        const double interval = c->lastSendStart ? static_cast<double>(started - c->lastSendStart) / 1e6 : -1.;
        c->lastSendStart = started;
        c->lastSendEnd = finished;
        c->lastInterval = interval;
        c->lastDuration = static_cast<double>(finished - started) / 1e6;
        ++c->sendAttempts;
        ++c->sendSuccesses;
        *timing = {static_cast<int64_t>(c->sendAttempts), started, finished, interval, c->lastDuration};
    });
    return ok && expired ? 2 : ok;
}
