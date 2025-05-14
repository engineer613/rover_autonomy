#ifndef ROVER_AUTONOMY_UM982OBSERVERS_HPP
#define ROVER_AUTONOMY_UM982OBSERVERS_HPP

template <typename MSG_TYPE>
class UnicoreObserver {
public:
  virtual ~UnicoreObserver() = default;
  virtual void notify(const MSG_TYPE& msg) = 0;
};

#endif // ROVER_AUTONOMY_UM982OBSERVERS_HPP
