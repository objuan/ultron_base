
#ifndef HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H
#define HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H

#include "ultron_kernel/horizon_legacy/clearpath.h"
#include "boost/type_traits/is_base_of.hpp"

namespace
{
  const uint16_t UNSUBSCRIBE = 0xFFFF;
}

namespace horizon_legacy
{

  void connect(std::string port);

  void reconnect();

  void configureLimits(double max_speed, double max_accel);

  void controlSpeed(double speed_left, double speed_right, double accel_left, double accel_right);

  template<typename T>
  struct Channel
  {

    typedef boost::shared_ptr<T> Ptr;
    typedef boost::shared_ptr<const T> ConstPtr;
    BOOST_STATIC_ASSERT_MSG(
      (boost::is_base_of<clearpath::Message, T>::value),
      "T must be a descendant of clearpath::Message"
    );

    static Ptr getLatest(double timeout)
    {
      T *latest = 0;

      // Iterate over all messages in queue and find the latest
      while (T *next = T::popNext())
      {
        if (latest)
        {
          delete latest;
          latest = 0;
        }
        latest = next;
      }

      // If no messages found in queue, then poll for timeout until one is received
      if (!latest)
      {
        latest = T::waitNext(timeout);
      }

      // If no messages received within timeout, make a request
      if (!latest)
      {
        return requestData(timeout);
      }

      return Ptr(latest);
    }

    static Ptr requestData(double timeout)
    {
      T *update = 0;
      while (!update)
      {
        update = T::getUpdate(timeout);
        if (!update)
        {
          reconnect();
        }
      }
      return Ptr(update);
    }

    static void subscribe(double frequency)
    {
      T::subscribe(frequency);
    }

    static void unsubscribe()
    {
      T::subscribe(UNSUBSCRIBE);
    }

  };

} // namespace ultron_kernel
#endif  // HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H
