#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <zmq.hpp>

#include "messages.pb.h"
#include "middleware/middleware.hpp"
#include "services/zmq_bridge_service.h"
#include "utils/logger.h"

namespace {

struct Ping {
  int id = 0;
  std::string payload;
};

class PubService : public adas::Service {
public:
  std::string_view getName() const override { return "pub"; }

  void configure() override {}

  void publishPing(int id, const std::string& payload)
  {
    Ping m{id, payload};
    publish("test/ping", m);
  }
};

class SubService : public adas::Service {
public:
  std::string_view getName() const override { return "sub"; }

  void configure() override
  {
    subscribe<Ping>("test/ping", [this](const Ping& m) {
      last_id_ = m.id;
      last_payload_ = m.payload;
      ++count_;
    });
  }

  void reset() override
  {
    count_ = 0;
    last_id_ = -1;
    last_payload_.clear();
  }

  int count() const { return count_.load(); }
  int lastId() const { return last_id_.load(); }
  std::string lastPayload() const { return last_payload_; }

private:
  std::atomic<int> count_{0};
  std::atomic<int> last_id_{-1};
  std::string last_payload_;
};

class TimerService : public adas::Service {
public:
  std::string_view getName() const override { return "timer"; }

  void configure() override
  {
    scheduleTimer(
        10, [this] { ++ticks_; }, "fast");
    scheduleTimer(
        50, [this] { ++slow_ticks_; }, "slow");
  }

  int ticks() const { return ticks_.load(); }
  int slowTicks() const { return slow_ticks_.load(); }

private:
  std::atomic<int> ticks_{0};
  std::atomic<int> slow_ticks_{0};
};

}  // namespace

TEST(MiddlewareTest, StartStopRealtime)
{
  auto pub = std::make_shared<PubService>();
  auto sub = std::make_shared<SubService>();
  auto mgr =
      std::make_shared<adas::Middleware>(adas::Middleware::Mode::RealTime, std::vector<adas::ServicePtr>{pub, sub});

  EXPECT_EQ(2u, mgr->startAll());
  EXPECT_TRUE(mgr->isRunning(pub));
  EXPECT_TRUE(mgr->isRunning(sub));

  pub->publishPing(7, "hello");
  for (int i = 0; i < 50 && sub->count() == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

  EXPECT_EQ(1, sub->count());
  EXPECT_EQ(7, sub->lastId());
  EXPECT_EQ("hello", sub->lastPayload());

  EXPECT_EQ(2u, mgr->stopAll());
  EXPECT_FALSE(mgr->isRunning(pub));
}

TEST(MiddlewareTest, TimersRealtime)
{
  auto svc = std::make_shared<TimerService>();
  auto mgr = std::make_shared<adas::Middleware>(adas::Middleware::Mode::RealTime, std::vector<adas::ServicePtr>{svc});
  ASSERT_EQ(1u, mgr->startAll());
  std::this_thread::sleep_for(std::chrono::milliseconds(55));
  EXPECT_GE(svc->ticks(), 3);
  mgr->stopAll();
}

TEST(MiddlewareTest, SimulatedStep)
{
  auto pub = std::make_shared<PubService>();
  auto sub = std::make_shared<SubService>();
  auto mgr =
      std::make_shared<adas::Middleware>(adas::Middleware::Mode::Simulated, std::vector<adas::ServicePtr>{pub, sub});

  mgr->setTime(0);
  pub->publishPing(1, "sim");
  EXPECT_EQ(0, sub->count());
  mgr->step();
  EXPECT_EQ(1, sub->count());
  EXPECT_EQ(1, sub->lastId());
}

TEST(MiddlewareTest, SimulatedTimers)
{
  auto svc = std::make_shared<TimerService>();
  auto mgr = std::make_shared<adas::Middleware>(adas::Middleware::Mode::Simulated, std::vector<adas::ServicePtr>{svc});

  mgr->setTime(0);
  mgr->step();
  EXPECT_EQ(0, svc->ticks());
  mgr->setTime(10'000);
  mgr->step();
  EXPECT_EQ(1, svc->ticks());
  mgr->setTime(30'000);
  mgr->step();
  EXPECT_GE(svc->ticks(), 3);
}

TEST(MiddlewareTest, InternalTopicPublishing)
{
  class CanMsg {
  public:
    uint32_t address = 0;
  };

  class Publisher : public adas::Service {
  public:
    void configure() override {}
    void send()
    {
      CanMsg m;
      m.address = 0xFD;
      publish("sensors/can", m);
    }
  };

  class Consumer : public adas::Service {
  public:
    void configure() override
    {
      subscribe<CanMsg>("sensors/can", [this](const CanMsg& m) {
        last_ = m.address;
        ++n_;
      });
    }
    std::atomic<int> n_{0};
    std::atomic<uint32_t> last_{0};
  };

  auto pub = std::make_shared<Publisher>();
  auto cons = std::make_shared<Consumer>();
  auto mgr =
      std::make_shared<adas::Middleware>(adas::Middleware::Mode::RealTime, std::vector<adas::ServicePtr>{pub, cons});
  mgr->startAll();
  pub->send();
  for (int i = 0; i < 50 && cons->n_ == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_EQ(1, cons->n_.load());
  EXPECT_EQ(0xFDu, cons->last_.load());
  mgr->stopAll();
}

TEST(MiddlewareTest, RegisterService)
{
  auto mw = std::make_shared<adas::Middleware>(adas::Middleware::Mode::Simulated);
  auto pub = mw->registerService<PubService>();
  auto sub = mw->registerService<SubService>();
  EXPECT_EQ(2u, mw->getServiceCount());

  mw->setTime(0);
  pub->publishPing(42, "reg");
  mw->step();
  EXPECT_EQ(1, sub->count());
  EXPECT_EQ(42, sub->lastId());
}

TEST(MiddlewareTest, BoundedSubscriptionDropsOldest)
{
  class SlowSub : public adas::Service {
  public:
    void configure() override
    {
      subscribe<Ping>(
          "test/ping", [this](const Ping& m) { ids_.push_back(m.id); },
          /*queue_capacity=*/2);
    }
    std::vector<int> ids_;
  };

  auto mw = std::make_shared<adas::Middleware>(adas::Middleware::Mode::Simulated);
  auto pub = mw->registerService<PubService>();
  auto sub = mw->registerService<SlowSub>();

  mw->setTime(0);
  for (int i = 1; i <= 5; ++i)
    pub->publishPing(i, "x");

  EXPECT_EQ(3u, mw->droppedTotal());
  mw->step();
  ASSERT_EQ(2u, sub->ids_.size());
  EXPECT_EQ(4, sub->ids_[0]);
  EXPECT_EQ(5, sub->ids_[1]);
}

TEST(MiddlewareTest, SnapshotStatsTracksTimers)
{
  auto mw = std::make_shared<adas::Middleware>(adas::Middleware::Mode::Simulated);
  auto svc = mw->registerService<TimerService>();

  for (int64_t t_us = 0; t_us <= 50'000; t_us += 10'000) {
    mw->setTime(t_us);
    mw->step();
  }

  const auto snap = mw->snapshotStats();
  EXPECT_EQ(1u, snap.services);
  ASSERT_FALSE(snap.services_timing.empty());
  EXPECT_GE(snap.services_timing[0].timers_fired, 3u);
  EXPECT_FLOAT_EQ(10.f, snap.services_timing[0].period_ms);
  ASSERT_EQ(2u, snap.services_timing[0].timers.size());

  EXPECT_EQ("fast", snap.services_timing[0].timers[0].name);
  EXPECT_FLOAT_EQ(10.f, snap.services_timing[0].timers[0].period_ms);
  EXPECT_EQ("slow", snap.services_timing[0].timers[1].name);
  EXPECT_FLOAT_EQ(50.f, snap.services_timing[0].timers[1].period_ms);

  EXPECT_FALSE(snap.services_timing[0].timers[0].lagging);
  EXPECT_FALSE(snap.any_lagging);
}

TEST(MiddlewareTest, ZmqBridgeForwardsOutboundTopicToSubscriber)
{
  const std::string ep_in = "tcp://127.0.0.1:5591";
  const std::string ep_out = "tcp://127.0.0.1:5592";

  auto mw = std::make_shared<adas::Middleware>(adas::Middleware::Mode::Simulated);
  mw->registerService<ZmqBridgeService>(ZmqBridgeService::Config{ep_in, ep_out});

  zmq::context_t ctx{1};
  zmq::socket_t sub{ctx, ZMQ_SUB};
  sub.set(zmq::sockopt::subscribe, "");
  sub.set(zmq::sockopt::rcvtimeo, 100);
  sub.connect(ep_out);

  ai::flow::adas::ZMQMessage out;
  out.set_topic("safety/warn");
  out.mutable_safety_warn()->set_fcw(true);
  out.mutable_safety_warn()->set_ttc_s(1.75f);

  ai::flow::adas::ZMQMessage got;
  bool received = false;
  for (int attempt = 0; attempt < 100 && !received; ++attempt) {
    mw->setTime(attempt * 10'000);
    mw->publish("safety/warn", out);
    mw->step();

    zmq::message_t frame;
    while (sub.recv(frame, zmq::recv_flags::dontwait)) {
      if (got.ParseFromArray(frame.data(), static_cast<int>(frame.size())) && got.has_safety_warn()) {
        received = true;
        break;
      }
    }
    if (!received)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(received) << "outbound topic never reached the ZMQ subscriber";
  EXPECT_EQ("safety/warn", got.topic());
  EXPECT_TRUE(got.safety_warn().fcw());
  EXPECT_FLOAT_EQ(1.75f, got.safety_warn().ttc_s());
}

namespace {

class KnobService : public adas::Service {
public:
  std::string_view getName() const override { return "knobs"; }

  void configure() override
  {
    registerParameter<double>("gain", gain);
    registerParameter<std::string>("mode", mode);
    registerParameter<bool>("on", on);
    registerParameter<double>(
        "clamped", [this](const double& v) { clamped = std::clamp(v, 0.0, 1.0); }, [this] { return clamped; });
    scheduleTimer(
        5, [this] { ticks.fetch_add(1); }, "tick");
  }

  double gain = 1.0;
  double clamped = 0.5;
  std::string mode = "pp";
  bool on = false;
  std::atomic<int> ticks{0};
};

}  // namespace

TEST(MiddlewareParams, SetByNameLandsOnTheServiceThread)
{
  auto svc = std::make_shared<KnobService>();
  adas::Middleware mw(adas::Middleware::Mode::RealTime, {svc});

  EXPECT_EQ(1u, mw.setParameter("gain", "2.5"));
  EXPECT_DOUBLE_EQ(1.0, svc->gain) << "must not apply on a foreign thread";

  mw.startAll();
  for (int i = 0; i < 200 && svc->gain != 2.5; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_DOUBLE_EQ(2.5, svc->gain);

  EXPECT_EQ(1u, mw.setParameter("mode", "fp"));
  EXPECT_EQ(1u, mw.setParameter("on", "true"));
  EXPECT_EQ(1u, mw.setParameter("clamped", "3.0"));
  // Wait for all three, not just the first: parked values may be applied in separate passes, so
  // watching one of them says nothing about the others.
  for (int i = 0; i < 200 && !(svc->mode == "fp" && svc->on && svc->clamped == 1.0); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_EQ("fp", svc->mode);
  EXPECT_TRUE(svc->on);
  EXPECT_DOUBLE_EQ(1.0, svc->clamped) << "setter must clamp";
  mw.stopAll();
}

TEST(MiddlewareParams, UnknownNameAndBadValueAreRejected)
{
  auto svc = std::make_shared<KnobService>();
  adas::Middleware mw(adas::Middleware::Mode::Simulated, {svc});

  EXPECT_EQ(0u, mw.setParameter("no_such", "1.0"));
  EXPECT_FALSE(mw.setParameter("knobs", "no_such", "1.0"));
  EXPECT_FALSE(mw.setParameter("no_such_service", "gain", "1.0"));
  EXPECT_FALSE(mw.setParameter("knobs", "gain", "not_a_number")) << "garbage must not overwrite the knob";

  EXPECT_TRUE(mw.setParameter("knobs", "gain", "4.0"));
  mw.step();
  EXPECT_DOUBLE_EQ(4.0, svc->gain) << "in simulation, step() applies";
  EXPECT_EQ("4.000000", mw.getParameter("gain"));
  EXPECT_TRUE(mw.getParameter("no_such").empty());
}

TEST(MiddlewareParams, OneNameCanLiveInSeveralServices)
{
  auto a = std::make_shared<KnobService>();
  auto b = std::make_shared<KnobService>();
  adas::Middleware mw(adas::Middleware::Mode::Simulated, {a, b});

  EXPECT_EQ(2u, mw.setParameter("gain", "7.0"));
  mw.step();
  EXPECT_DOUBLE_EQ(7.0, a->gain);
  EXPECT_DOUBLE_EQ(7.0, b->gain);

  const auto names = mw.parameterNames();
  ASSERT_EQ(1u, names.count("knobs"));
  EXPECT_EQ(4u, names.at("knobs").size());
}

TEST(MiddlewareParams, DuplicateRegistrationIsRefused)
{
  class Twice : public adas::Service {
  public:
    std::string_view getName() const override { return "twice"; }
    void configure() override
    {
      first = registerParameter<double>("gain", gain);
      second = registerParameter<double>("gain", other);
    }
    double gain = 0.0, other = 0.0;
    bool first = false, second = true;
  };

  auto svc = std::make_shared<Twice>();
  adas::Middleware mw(adas::Middleware::Mode::Simulated, {svc});
  EXPECT_TRUE(svc->first);
  EXPECT_FALSE(svc->second) << "duplicate name must be rejected or one field silently wins";
}
