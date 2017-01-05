
#ifndef ultron_kernel_ultron_DIAGNOSTICS_H
#define ultron_kernel_ultron_DIAGNOSTICS_H

#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "ultron_kernel/horizon_legacy_wrapper.h"
#include "ultron_kernel/RobotStatus.h"

namespace ultron_kernel
{

  class UltronSoftwareDiagnosticTask :
    public diagnostic_updater::DiagnosticTask
  {
  public:
    explicit UltronSoftwareDiagnosticTask(ultron_kernel::RobotStatus &msg, double target_control_freq);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void updateControlFrequency(double frequency);

  private:
    void reset();

    double control_freq_, target_control_freq_;
    ultron_kernel::RobotStatus &msg_;
  };

  template<typename T>
  class UltronHardwareDiagnosticTask :
    public diagnostic_updater::DiagnosticTask
  {
  public:
    explicit UltronHardwareDiagnosticTask(ultron_kernel::RobotStatus &msg);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      typename horizon_legacy::Channel<T>::Ptr latest = horizon_legacy::Channel<T>::requestData(1.0);
      if (latest)
      {
        update(stat, latest);
      }
    }

    void update(diagnostic_updater::DiagnosticStatusWrapper &stat, typename horizon_legacy::Channel<T>::Ptr &status);

  private:
    ultron_kernel::RobotStatus &msg_;
  };

  template<>
  UltronHardwareDiagnosticTask<clearpath::DataSystemStatus>::UltronHardwareDiagnosticTask(ultron_kernel::RobotStatus &msg);

  template<>
  UltronHardwareDiagnosticTask<clearpath::DataPowerSystem>::UltronHardwareDiagnosticTask(ultron_kernel::RobotStatus &msg);

  template<>
  UltronHardwareDiagnosticTask<clearpath::DataSafetySystemStatus>::UltronHardwareDiagnosticTask(
    ultron_kernel::RobotStatus &msg);

  template<>
  void UltronHardwareDiagnosticTask<clearpath::DataSystemStatus>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    horizon_legacy::Channel<clearpath::DataSystemStatus>::Ptr &status);

  template<>
  void UltronHardwareDiagnosticTask<clearpath::DataPowerSystem>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    horizon_legacy::Channel<clearpath::DataPowerSystem>::Ptr &status);

  template<>
  void UltronHardwareDiagnosticTask<clearpath::DataSafetySystemStatus>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    horizon_legacy::Channel<clearpath::DataSafetySystemStatus>::Ptr &status);

}  // namespace ultron_kernel
#endif  // ultron_kernel_ultron_DIAGNOSTICS_H
