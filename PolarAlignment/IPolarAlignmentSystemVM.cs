using System.Threading;
using System.Threading.Tasks;

namespace NINA.Plugins.PolarAlignment {
    public interface IPolarAlignmentSystemVM {
        bool Connected { get; }
        bool DoAutomatedAdjustments { get; set; }
        double AutomatedAdjustmentSettleTime { get; set; }

        Task Connect();
        void Disconnect();
        Task<bool> TryNudgeX(float position, CancellationToken token);
        Task<bool> TryNudgeY(float position, CancellationToken token);
        Task NudgeX(float position, CancellationToken token);
        Task NudgeY(float position, CancellationToken token);
        void RaiseAllPropertiesChanged();
    }
}
