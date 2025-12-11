#ifndef DWAPATH_H
#define DWAPATH_H

#include <Thread.h>
#include <vector>
#include <Vector2D.h>

namespace flair {
    namespace gui {
        class PushButton;
        class DoubleSpinBox;
    }
    namespace filter {
        class dwa2Dtrajectory;
        class Pid;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
}

class dwa_path : public flair::core::Thread {
public:
    dwa_path(std::string name, flair::sensor::TargetController *controller);
    ~dwa_path();

private:
    enum class BehaviourMode_t {
        Manual,
        GotoGoal  // ✓ Cohérent avec l'implémentation
    };

    void Run(void) override;
    void StartDriving(void);
    void StopDriving(void);
    void ComputeManualControls(void);
    void ComputeGotoGoalControls(void);
    void SecurityCheck(void);
    void CheckJoystick(void);
    void CheckPushButton(void);

    // Contrôleurs PID
    flair::filter::Pid *uX, *uY;
    
    // Boutons UI
    flair::gui::PushButton *startGoal, *stopGoal;  // ✓ Noms cohérents
    flair::gui::PushButton *quitProgram, *startLog, *stopLog;
    
    // Paramètres
    flair::gui::DoubleSpinBox *l;
    
    // Objets VRPN
    flair::meta::MetaVrpnObject *targetVrpn, *ugvVrpn;
    
    // Planificateur DWA
    flair::filter::dwa2Dtrajectory *trajectory;
    
    // État
    BehaviourMode_t behaviourMode;
    bool vrpnLost;
    
    // Contrôleur joystick
    flair::sensor::TargetController *controller;
};

#endif // DWAPATH_H