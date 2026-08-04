#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "utils/logger.h"

namespace adas {

namespace detail {

// Free function (not a class static): NDK clang fails to link static template members
// used above their definition.
template <typename T>
inline bool parseParamText(const std::string& text, T& out)
{
  if constexpr (std::is_same_v<T, std::string>) {
    out = text;
    return true;
  } else if constexpr (std::is_same_v<T, bool>) {
    if (text == "true" || text == "1") {
      out = true;
      return true;
    }
    if (text == "false" || text == "0") {
      out = false;
      return true;
    }
    return false;
  } else {
    try {
      if constexpr (std::is_same_v<T, int>)
        out = std::stoi(text);
      else if constexpr (std::is_same_v<T, long long>)
        out = std::stoll(text);
      else if constexpr (std::is_same_v<T, float>)
        out = std::stof(text);
      else
        out = std::stod(text);
    } catch (const std::exception&) {
      return false;
    }
    return true;
  }
}

}  // namespace detail

class Middleware;
class Service;

using ServicePtr = std::shared_ptr<Service>;

inline constexpr std::size_t kDefaultSubQueueCapacity = 100;

struct TimerSnapshot {
  std::string name;
  float period_ms = 0;
  float last_dt_ms = 0;
  float mean_dt_ms = 0;
  float max_dt_ms = 0;
  bool lagging = false;
  uint64_t fired = 0;
};

struct ServiceSnapshot {
  std::string name;
  bool running = false;
  uint64_t messages_processed = 0;
  uint64_t timers_fired = 0;
  uint64_t exceptions = 0;
  uint64_t dropped = 0;
  uint32_t inbox_depth = 0;
  uint32_t backlog_depth = 0;
  float last_cb_ms = 0;
  float mean_cb_ms = 0;
  float max_cb_ms = 0;

  float period_ms = 0;
  float last_dt_ms = 0;
  float mean_dt_ms = 0;
  float max_dt_ms = 0;
  bool lagging = false;
  std::vector<TimerSnapshot> timers;
};

struct MiddlewareSnapshot {
  uint64_t timestamp_us = 0;
  uint64_t dropped_total = 0;
  uint32_t services = 0;
  uint32_t running = 0;
  bool any_lagging = false;
  std::vector<ServiceSnapshot> services_timing;
};

class Service : public std::enable_shared_from_this<Service> {
public:
  virtual ~Service() = default;

  virtual std::string_view getName() const { return "service"; }

protected:
  template <typename T>
  void publish(const std::string& topic, const T& msg);

  template <typename T>
  void subscribe(const std::string& topic, std::function<void(const T&)> cb,
                 std::size_t queue_capacity = kDefaultSubQueueCapacity);

  void scheduleTimer(uint64_t interval_ms, std::function<void()> cb, std::string name = {});

  template <typename T>
  bool registerParameter(const std::string& name, T& field);

  template <typename T>
  bool registerParameter(const std::string& name, std::function<void(const T&)> setter, std::function<T(void)> getter);

  size_t getQueueSize() const;

  uint64_t now() const;

  Middleware* middleware() const { return bus_; }

  virtual void configure() = 0;
  virtual void reset() {}

  virtual void onException(const std::exception& e, const std::string& context)
  {
    LOGE("Service[%s] %s: %s", std::string(getName()).c_str(), context.c_str(), e.what());
  }

  friend class Middleware;

private:
  Middleware* bus_ = nullptr;
  size_t slot_ = static_cast<size_t>(-1);
};

class Middleware {
public:
  enum class Mode { RealTime, Simulated };

  explicit Middleware(Mode mode) : mode_(mode) {}

  Middleware(Mode mode, const std::vector<ServicePtr>& services) : Middleware(mode)
  {
    for (const auto& svc : services)
      registerService(svc);
  }

  ~Middleware() { stopAll(); }

  Middleware(const Middleware&) = delete;
  Middleware& operator=(const Middleware&) = delete;

  void registerService(const ServicePtr& svc)
  {
    if (!svc)
      throw std::invalid_argument("registerService: null service");
    if (svc->bus_ != nullptr)
      throw std::invalid_argument("registerService: service already attached");

    auto slot = std::make_unique<Slot>();
    slot->service = svc;
    const size_t idx = slots_.size();
    svc->bus_ = this;
    svc->slot_ = idx;
    slots_.push_back(std::move(slot));
    svc->configure();
  }

  template <typename ServiceType, typename... Args>
  std::shared_ptr<ServiceType> registerService(Args&&... args)
  {
    static_assert(std::is_base_of<Service, ServiceType>::value);
    auto svc = std::make_shared<ServiceType>(std::forward<Args>(args)...);
    registerService(std::static_pointer_cast<Service>(svc));
    return svc;
  }

  bool start(const ServicePtr& svc)
  {
    if (mode_ != Mode::RealTime || !svc)
      return false;
    Slot* slot = slotOf(svc.get());
    if (!slot || slot->alive.load())
      return false;

    svc->reset();
    slot->alive = true;
    slot->worker = std::thread([this, slot] { workerLoop(*slot); });
    return true;
  }

  bool stop(const ServicePtr& svc)
  {
    if (mode_ != Mode::RealTime || !svc)
      return false;
    Slot* slot = slotOf(svc.get());
    if (!slot || !slot->alive.load())
      return false;

    {
      std::lock_guard<std::mutex> lk(slot->mu);
      slot->alive = false;
    }
    slot->cv.notify_all();
    if (slot->worker.joinable())
      slot->worker.join();
    return true;
  }

  size_t startAll()
  {
    size_t n = 0;
    for (auto& slot : slots_) {
      if (start(slot->service))
        ++n;
    }
    return n;
  }

  size_t stopAll()
  {
    size_t n = 0;
    for (auto& slot : slots_) {
      if (stop(slot->service))
        ++n;
    }
    return n;
  }

  bool isRunning(const ServicePtr& svc) const
  {
    const Slot* slot = slotOf(svc.get());
    return slot && slot->alive.load();
  }

  size_t getServiceCount() const { return slots_.size(); }

  size_t getRunningCount() const
  {
    size_t n = 0;
    for (const auto& slot : slots_) {
      if (slot->alive.load())
        ++n;
    }
    return n;
  }

  uint64_t droppedTotal() const
  {
    uint64_t n = 0;
    std::lock_guard<std::mutex> lk(router_mu_);
    for (const auto& kv : router_) {
      for (const auto& sub : kv.second) {
        if (sub && sub->slot)
          n += sub->slot->dropped.load();
      }
    }
    return n;
  }

  MiddlewareSnapshot snapshotStats() const
  {
    MiddlewareSnapshot out;
    out.timestamp_us = now();
    out.dropped_total = droppedTotal();
    out.services = static_cast<uint32_t>(slots_.size());
    out.running = static_cast<uint32_t>(getRunningCount());

    std::vector<std::shared_ptr<MessageSlot>> msg_slots;
    {
      std::lock_guard<std::mutex> lk(router_mu_);
      for (const auto& kv : router_) {
        for (const auto& sub : kv.second) {
          if (sub && sub->slot)
            msg_slots.push_back(sub->slot);
        }
      }
    }

    std::vector<uint32_t> backlog(slots_.size(), 0);
    std::vector<uint64_t> dropped(slots_.size(), 0);
    for (const auto& ms : msg_slots) {
      if (ms->service_slot >= backlog.size())
        continue;
      std::lock_guard<std::mutex> sl(ms->mu);
      backlog[ms->service_slot] += static_cast<uint32_t>(ms->backlog.size());
      dropped[ms->service_slot] += ms->dropped.load();
    }

    out.services_timing.reserve(slots_.size());
    for (size_t i = 0; i < slots_.size(); ++i) {
      const Slot& slot = *slots_[i];
      ServiceSnapshot s;
      s.name = slot.service ? std::string(slot.service->getName()) : "?";
      s.running = slot.alive.load();
      s.dropped = dropped[i];
      s.backlog_depth = backlog[i];
      {
        std::lock_guard<std::mutex> lk(slot.mu);
        s.inbox_depth = static_cast<uint32_t>(slot.inbox.size());
        s.messages_processed = slot.stats.messages_processed;
        s.timers_fired = slot.stats.timers_fired;
        s.exceptions = slot.stats.exceptions;
        s.last_cb_ms = static_cast<float>(slot.stats.cb_us.last() / 1000.0);
        s.mean_cb_ms = static_cast<float>(slot.stats.cb_us.mean() / 1000.0);
        s.max_cb_ms = static_cast<float>(slot.stats.cb_us.max() / 1000.0);
        s.period_ms = static_cast<float>(slot.stats.min_period_us / 1000.0);

        const Periodic* shortest = nullptr;
        s.timers.reserve(slot.periodics.size());
        for (const auto& p : slot.periodics) {
          TimerSnapshot t;
          t.name = p.name.empty() ? (std::to_string(p.period_us / 1000) + "ms") : p.name;
          t.period_ms = static_cast<float>(p.period_us / 1000.0);
          t.last_dt_ms = static_cast<float>(p.dt_us.last() / 1000.0);
          t.mean_dt_ms = static_cast<float>(p.dt_us.mean() / 1000.0);
          t.max_dt_ms = static_cast<float>(p.dt_us.max() / 1000.0);
          t.fired = p.fired;
          if (p.period_us > 0 && p.dt_us.count() > 0) {
            const double expected = static_cast<double>(p.period_us) / 0.9;
            t.lagging = p.dt_us.mean() > expected;
          }
          if (t.lagging)
            s.lagging = true;
          if (!shortest || p.period_us < shortest->period_us)
            shortest = &p;
          s.timers.push_back(std::move(t));
        }
        if (shortest && shortest->dt_us.count() > 0) {
          s.last_dt_ms = static_cast<float>(shortest->dt_us.last() / 1000.0);
          s.mean_dt_ms = static_cast<float>(shortest->dt_us.mean() / 1000.0);
          s.max_dt_ms = static_cast<float>(shortest->dt_us.max() / 1000.0);
        }
      }
      if (s.lagging)
        out.any_lagging = true;
      out.services_timing.push_back(std::move(s));
    }
    return out;
  }

  void printStats() const
  {
    const auto snap = snapshotStats();
    LOGI("Middleware services=%u running=%u dropped=%llu lagging=%d mode=%s", snap.services, snap.running,
         static_cast<unsigned long long>(snap.dropped_total), snap.any_lagging ? 1 : 0,
         mode_ == Mode::RealTime ? "realtime" : "simulated");
    for (const auto& s : snap.services_timing) {
      if (s.messages_processed == 0 && s.timers_fired == 0 && s.dropped == 0)
        continue;
      LOGI("  %-16s msg=%llu tim=%llu drop=%llu cb_ms=%.2f/%.2f lag=%d", s.name.c_str(),
           static_cast<unsigned long long>(s.messages_processed), static_cast<unsigned long long>(s.timers_fired),
           static_cast<unsigned long long>(s.dropped), s.mean_cb_ms, s.max_cb_ms, s.lagging ? 1 : 0);
      for (const auto& t : s.timers) {
        LOGI("    timer %-12s period=%.1f mean_dt=%.2f max_dt=%.2f lag=%d fired=%llu", t.name.c_str(), t.period_ms,
             t.mean_dt_ms, t.max_dt_ms, t.lagging ? 1 : 0, static_cast<unsigned long long>(t.fired));
      }
    }
  }

  void setTime(uint64_t t_us)
  {
    if (mode_ == Mode::Simulated)
      sim_us_ = t_us;
  }

  void step()
  {
    if (mode_ != Mode::Simulated)
      return;

    for (auto& slot : slots_)
      applyParams(*slot);

    for (;;) {
      bool any = false;
      for (auto& slot : slots_) {
        if (drainInbox(*slot))
          any = true;
      }
      if (!any)
        break;
    }

    const uint64_t t = now();
    for (auto& slot : slots_)
      fireDueTimers(*slot, t);
  }

  template <typename T>
  void publish(const std::string& topic, const T& msg)
  {
    std::vector<std::shared_ptr<SubscriptionBase>> subs;
    {
      std::lock_guard<std::mutex> lk(router_mu_);
      auto it = router_.find(topic);
      if (it == router_.end())
        return;
      subs = it->second;
    }
    for (const auto& sub : subs) {
      if (!sub)
        continue;
      if (sub->messageType() != typeid(T)) {
        LOGE("publish type mismatch on '%s': published=%s subscriber=%s", topic.c_str(), typeid(T).name(),
             sub->messageType().name());
        continue;
      }
      sub->publishCopy(&msg);
    }
  }

  template <typename T>
  void subscribe(const std::string& topic, const ServicePtr& svc, std::function<void(const T&)> cb,
                 std::size_t queue_capacity = kDefaultSubQueueCapacity)
  {
    if (!svc)
      throw std::invalid_argument("subscribe: null service");
    Slot* slot = slotOf(svc.get());
    if (!slot)
      throw std::runtime_error("subscribe: service not registered");
    if (queue_capacity == 0)
      throw std::invalid_argument("subscribe: queue_capacity must be > 0");

    auto msg_slot = std::make_shared<MessageSlot>();
    msg_slot->owner = this;
    msg_slot->service_slot = svc->slot_;
    msg_slot->capacity = queue_capacity;

    auto sub = std::make_shared<Subscription<T>>();
    sub->slot = msg_slot;
    sub->cb = std::move(cb);

    std::lock_guard<std::mutex> lk(router_mu_);
    router_[topic].push_back(std::move(sub));
  }

  template <typename T>
  bool registerParameter(Service* svc, const std::string& name, T& field)
  {
    Slot* slot = slotOf(svc);
    if (!slot)
      return false;
    std::lock_guard<std::mutex> lk(slot->param_mu);
    return slot->params.bind(name, field);
  }

  template <typename T>
  bool registerParameter(Service* svc, const std::string& name, std::function<void(const T&)> setter,
                         std::function<T(void)> getter)
  {
    Slot* slot = slotOf(svc);
    if (!slot)
      return false;
    std::lock_guard<std::mutex> lk(slot->param_mu);
    return slot->params.bind<T>(name, std::move(setter), std::move(getter));
  }

  bool setParameter(const std::string& service, const std::string& name, const std::string& value)
  {
    for (auto& slot : slots_) {
      if (!slot->service || std::string(slot->service->getName()) != service)
        continue;
      std::lock_guard<std::mutex> lk(slot->param_mu);
      const bool ok = slot->params.queue(name, value);
      if (ok && mode_ == Mode::RealTime)
        slot->cv.notify_one();
      return ok;
    }
    return false;
  }

  size_t setParameter(const std::string& name, const std::string& value)
  {
    size_t applied = 0;
    for (auto& slot : slots_) {
      std::lock_guard<std::mutex> lk(slot->param_mu);
      if (!slot->params.queue(name, value))
        continue;
      ++applied;
      if (mode_ == Mode::RealTime)
        slot->cv.notify_one();
    }
    return applied;
  }

  std::string getParameter(const std::string& name) const
  {
    for (const auto& slot : slots_) {
      std::lock_guard<std::mutex> lk(slot->param_mu);
      if (slot->params.has(name))
        return slot->params.read(name);
    }
    return {};
  }

  std::map<std::string, std::set<std::string>> parameterNames() const
  {
    std::map<std::string, std::set<std::string>> out;
    for (const auto& slot : slots_) {
      if (!slot->service)
        continue;
      std::lock_guard<std::mutex> lk(slot->param_mu);
      auto names = slot->params.keys();
      if (!names.empty())
        out[std::string(slot->service->getName())] = std::move(names);
    }
    return out;
  }

  void scheduleTimer(uint64_t interval_ms, const ServicePtr& svc, std::function<void()> cb, std::string name = {})
  {
    if (!svc)
      throw std::invalid_argument("scheduleTimer: null service");
    if (interval_ms == 0)
      throw std::invalid_argument("scheduleTimer: interval must be > 0");
    Slot* slot = slotOf(svc.get());
    if (!slot)
      throw std::runtime_error("scheduleTimer: service not registered");

    const uint64_t interval_us = interval_ms * 1000ULL;
    Periodic p;
    p.period_us = interval_us;
    p.next_us = now() + interval_us;
    p.fn = std::move(cb);
    p.name = std::move(name);
    if (p.name.empty())
      p.name = std::to_string(interval_ms) + "ms";

    {
      std::lock_guard<std::mutex> lk(slot->mu);
      slot->periodics.push_back(std::move(p));
      if (slot->stats.min_period_us == 0 || interval_us < slot->stats.min_period_us)
        slot->stats.min_period_us = interval_us;
    }
    slot->cv.notify_one();
  }

  size_t getQueueSize(const ServicePtr& svc) const
  {
    const Slot* slot = slotOf(svc.get());
    if (!slot)
      throw std::runtime_error("getQueueSize: service not registered");
    std::lock_guard<std::mutex> lk(slot->mu);
    return slot->inbox.size();
  }

  uint64_t now() const
  {
    if (mode_ == Mode::Simulated)
      return sim_us_;

    struct timespec ts {};
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1'000ULL;
  }

private:
  using Job = std::function<void()>;

  // Per-service knobs: writers only queue; the service thread flushes between callbacks.
  class ParamBag {
  public:
    template <typename T>
    struct Entry {
      using Type = T;
      T pending{};
      bool dirty = false;
      std::function<void(const T&)> set{};
      std::function<T()> get{};
    };

    using EntryVariant =
        std::variant<Entry<bool>, Entry<int>, Entry<long long>, Entry<float>, Entry<double>, Entry<std::string>>;

    template <typename T>
    bool bind(const std::string& name, std::function<void(const T&)> set, std::function<T()> get)
    {
      if (!set || !get || entries_.count(name) != 0)
        return false;
      entries_.emplace(name, Entry<T>{get(), false, std::move(set), std::move(get)});
      return true;
    }

    template <typename T>
    bool bind(const std::string& name, T& field)
    {
      return bind<T>(
          name, [&field](const T& v) { field = v; }, [&field] { return field; });
    }

    bool queue(const std::string& name, const std::string& text)
    {
      const auto it = entries_.find(name);
      if (it == entries_.end())
        return false;
      return std::visit(
          [&](auto& e) -> bool {
            using T = typename std::decay_t<decltype(e)>::Type;
            T parsed{};
            if (!detail::parseParamText(text, parsed))
              return false;
            e.pending = std::move(parsed);
            e.dirty = true;
            pending_ = true;
            return true;
          },
          it->second);
    }

    bool has(const std::string& name) const { return entries_.count(name) != 0; }

    std::string read(const std::string& name) const
    {
      const auto it = entries_.find(name);
      if (it == entries_.end())
        throw std::runtime_error("no such parameter: " + name);
      return std::visit(
          [](const auto& e) -> std::string {
            using T = typename std::decay_t<decltype(e)>::Type;
            if constexpr (std::is_same_v<T, std::string>)
              return e.get();
            else if constexpr (std::is_same_v<T, bool>)
              return e.get() ? "true" : "false";
            else
              return std::to_string(e.get());
          },
          it->second);
    }

    std::set<std::string> keys() const
    {
      std::set<std::string> out;
      for (const auto& [name, _] : entries_)
        out.insert(name);
      return out;
    }

    bool pending() const { return pending_; }

    void flush()
    {
      for (auto& [_, entry] : entries_) {
        std::visit(
            [](auto& e) {
              if (!e.dirty)
                return;
              e.dirty = false;
              e.set(e.pending);
            },
            entry);
      }
      pending_ = false;
    }

    std::unordered_map<std::string, EntryVariant> entries_;
    bool pending_ = false;
  };

  struct RollingUs {
    static constexpr std::size_t kCap = 100;
    double buf[kCap]{};
    std::size_t n = 0;
    std::size_t i = 0;
    double last_v = 0;
    double max_v = 0;

    void add(double us)
    {
      last_v = us;
      if (us > max_v)
        max_v = us;
      buf[i % kCap] = us;
      ++i;
      if (n < kCap)
        ++n;
    }

    std::size_t count() const { return n; }
    double last() const { return last_v; }
    double max() const { return max_v; }
    double mean() const
    {
      if (n == 0)
        return 0;
      double s = 0;
      for (std::size_t k = 0; k < n; ++k)
        s += buf[k];
      return s / static_cast<double>(n);
    }
  };

  struct SlotStats {
    uint64_t messages_processed = 0;
    uint64_t timers_fired = 0;
    uint64_t exceptions = 0;
    uint64_t min_period_us = 0;
    RollingUs cb_us;
  };

  struct Periodic {
    std::string name;
    uint64_t period_us = 0;
    uint64_t next_us = 0;
    uint64_t last_fire_us = 0;
    uint64_t fired = 0;
    RollingUs dt_us;
    Job fn;
  };

  struct Slot {
    ServicePtr service;
    mutable std::mutex mu;

    mutable std::mutex param_mu;
    ParamBag params;
    std::condition_variable cv;
    std::deque<Job> inbox;
    std::vector<Periodic> periodics;
    std::thread worker;
    std::atomic<bool> alive{false};
    SlotStats stats;
  };

  struct MessageSlot : public std::enable_shared_from_this<MessageSlot> {
    Middleware* owner = nullptr;
    size_t service_slot = 0;
    std::size_t capacity = kDefaultSubQueueCapacity;

    mutable std::mutex mu;
    std::deque<std::function<void()>> backlog;
    bool drain_pending = false;
    std::atomic<uint64_t> dropped{0};

    void enqueueJob(std::function<void()> job)
    {
      if (!job)
        return;
      bool schedule = false;
      {
        std::lock_guard<std::mutex> lk(mu);
        backlog.push_back(std::move(job));
        while (backlog.size() > capacity) {
          backlog.pop_front();
          dropped.fetch_add(1, std::memory_order_relaxed);
        }
        if (!drain_pending) {
          drain_pending = true;
          schedule = true;
        }
      }
      if (schedule && owner)
        owner->scheduleDrain(shared_from_this());
    }

    void drainOne()
    {
      std::function<void()> job;
      {
        std::lock_guard<std::mutex> lk(mu);
        if (backlog.empty()) {
          drain_pending = false;
          return;
        }
        job = std::move(backlog.front());
        backlog.pop_front();
      }

      if (job) {
        job();
        if (owner && service_slot < owner->slots_.size()) {
          std::lock_guard<std::mutex> lk(owner->slots_[service_slot]->mu);
          owner->slots_[service_slot]->stats.messages_processed++;
        }
      }

      bool more = false;
      {
        std::lock_guard<std::mutex> lk(mu);
        if (!backlog.empty())
          more = true;
        else
          drain_pending = false;
      }
      if (more && owner)
        owner->scheduleDrain(shared_from_this());
    }
  };

  struct SubscriptionBase {
    virtual ~SubscriptionBase() = default;
    virtual const std::type_info& messageType() const = 0;

    virtual void publishCopy(const void* msg) = 0;
    std::shared_ptr<MessageSlot> slot;
  };

  template <typename T>
  struct Subscription final : SubscriptionBase {
    std::function<void(const T&)> cb;

    const std::type_info& messageType() const override { return typeid(T); }

    void publishCopy(const void* msg) override
    {
      if (!msg || !slot || !cb)
        return;
      auto held = std::make_shared<T>(*static_cast<const T*>(msg));
      auto callback = cb;
      slot->enqueueJob([callback, held]() { callback(*held); });
    }
  };

  void scheduleDrain(const std::shared_ptr<MessageSlot>& sub)
  {
    if (!sub || sub->service_slot >= slots_.size())
      return;
    Slot& slot = *slots_[sub->service_slot];
    {
      std::lock_guard<std::mutex> lk(slot.mu);
      slot.inbox.push_back([sub]() { sub->drainOne(); });
    }
    if (mode_ == Mode::RealTime)
      slot.cv.notify_one();
  }

  void applyParams(Slot& slot)
  {
    std::lock_guard<std::mutex> lk(slot.param_mu);
    if (slot.params.pending())
      slot.params.flush();
  }

  Slot* slotOf(Service* svc)
  {
    if (!svc || svc->slot_ >= slots_.size())
      return nullptr;
    Slot* s = slots_[svc->slot_].get();
    return (s && s->service.get() == svc) ? s : nullptr;
  }

  const Slot* slotOf(Service* svc) const { return const_cast<Middleware*>(this)->slotOf(svc); }

  bool drainInbox(Slot& slot)
  {
    std::deque<Job> batch;
    {
      std::lock_guard<std::mutex> lk(slot.mu);
      if (slot.inbox.empty())
        return false;
      batch.swap(slot.inbox);
    }
    for (auto& job : batch)
      invoke(slot, job, /*timer=*/false);
    return true;
  }

  void fireDueTimers(Slot& slot, uint64_t t_us)
  {
    std::vector<Job> due;
    {
      std::lock_guard<std::mutex> lk(slot.mu);
      for (auto& p : slot.periodics) {
        if (t_us < p.next_us)
          continue;

        if (p.last_fire_us > 0)
          p.dt_us.add(static_cast<double>(t_us - p.last_fire_us));
        p.last_fire_us = t_us;
        while (t_us >= p.next_us) {
          due.push_back(p.fn);
          p.next_us += p.period_us;
          p.fired++;
        }
      }
    }
    for (auto& job : due)
      invoke(slot, job, /*timer=*/true);
  }

  static void invoke(Slot& slot, Job& job, bool timer)
  {
    const uint64_t t0 = slot.service && slot.service->bus_ ? slot.service->bus_->now() : 0;
    try {
      job();
      const uint64_t t1 = slot.service && slot.service->bus_ ? slot.service->bus_->now() : t0;
      std::lock_guard<std::mutex> lk(slot.mu);
      slot.stats.cb_us.add(static_cast<double>(t1 - t0));
      if (timer)
        slot.stats.timers_fired++;
    } catch (const std::exception& e) {
      {
        std::lock_guard<std::mutex> lk(slot.mu);
        slot.stats.exceptions++;
      }
      if (slot.service)
        slot.service->onException(e, timer ? "timer" : "message");
    } catch (...) {
      {
        std::lock_guard<std::mutex> lk(slot.mu);
        slot.stats.exceptions++;
      }
      LOGE("Service[%s] unknown exception in %s", slot.service ? std::string(slot.service->getName()).c_str() : "?",
           timer ? "timer" : "message");
    }
  }

  void workerLoop(Slot& slot)
  {
    {
      const uint64_t t0 = now();
      std::lock_guard<std::mutex> lk(slot.mu);
      for (auto& p : slot.periodics)
        p.next_us = t0 + p.period_us;
    }

    while (slot.alive.load()) {
      applyParams(slot);
      drainInbox(slot);
      fireDueTimers(slot, now());

      std::unique_lock<std::mutex> lk(slot.mu);
      if (!slot.alive.load())
        break;
      if (!slot.inbox.empty())
        continue;

      const auto deadline = [&]() -> std::optional<uint64_t> {
        std::optional<uint64_t> soonest;
        for (const auto& p : slot.periodics) {
          if (!soonest || p.next_us < *soonest)
            soonest = p.next_us;
        }
        return soonest;
      }();

      if (deadline) {
        const uint64_t t = now();
        if (t >= *deadline)
          continue;
        slot.cv.wait_for(lk, std::chrono::microseconds(*deadline - t),
                         [&] { return !slot.alive.load() || !slot.inbox.empty(); });
      } else {
        slot.cv.wait(lk, [&] { return !slot.alive.load() || !slot.inbox.empty(); });
      }
    }

    drainInbox(slot);
  }

  Mode mode_;
  uint64_t sim_us_ = 0;
  std::vector<std::unique_ptr<Slot>> slots_;
  std::unordered_map<std::string, std::vector<std::shared_ptr<SubscriptionBase>>> router_;
  mutable std::mutex router_mu_;
};

template <typename T>
inline void Service::publish(const std::string& topic, const T& msg)
{
  if (!bus_)
    throw std::runtime_error("publish: service is not registered");
  bus_->publish(topic, msg);
}

template <typename T>
inline void Service::subscribe(const std::string& topic, std::function<void(const T&)> cb, std::size_t queue_capacity)
{
  if (!bus_)
    throw std::runtime_error("subscribe: service is not registered");
  bus_->subscribe(topic, shared_from_this(), std::move(cb), queue_capacity);
}

template <typename T>
inline bool Service::registerParameter(const std::string& name, T& field)
{
  return bus_ != nullptr && bus_->registerParameter<T>(this, name, field);
}

template <typename T>
inline bool Service::registerParameter(const std::string& name, std::function<void(const T&)> setter,
                                       std::function<T(void)> getter)
{
  return bus_ != nullptr && bus_->registerParameter<T>(this, name, std::move(setter), std::move(getter));
}

inline void Service::scheduleTimer(uint64_t interval_ms, std::function<void()> cb, std::string name)
{
  if (!bus_)
    throw std::runtime_error("scheduleTimer: service is not registered");
  bus_->scheduleTimer(interval_ms, shared_from_this(), std::move(cb), std::move(name));
}

inline size_t Service::getQueueSize() const
{
  if (!bus_)
    throw std::runtime_error("getQueueSize: service is not registered");
  return bus_->getQueueSize(std::const_pointer_cast<Service>(shared_from_this()));
}

inline uint64_t Service::now() const
{
  if (!bus_)
    throw std::runtime_error("now: service is not registered");
  return bus_->now();
}

}  // namespace adas
