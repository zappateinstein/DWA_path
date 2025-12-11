#include "dwapath.h"
#include "dwa2Dtrajectory.h"
#include <TargetController.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <Matrix.h>
#include <Tab.h>
#include <TabWidget.h>
#include <DoubleSpinBox.h>
#include <Pid.h>
#include <Quaternion.h>
#include <Euler.h>
#include <Ugv.h>
#include <UgvControls.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;
using namespace flair::actuator;

// ========== CONSTRUCTEUR ==========
dwa_path::dwa_path(string name, TargetController *controller)
        : Thread(getFrameworkManager(), "DWA_Controller", 50),
          behaviourMode(BehaviourMode_t::Manual),
          vrpnLost(false) {
    std::cout << "création du chemin suivi par le robot\n";
    this->controller = controller;
    controller->Start();

    Ugv* ugv = GetUgv();
    ugv->UseDefaultPlot();
    //std::cout << "Enrefistrement de l'UGV dans notre le GCS\n";
    // ========== VRPN Setup ==========
    VrpnClient* vrpnclient = new VrpnClient("vrpn", ugv->GetDefaultVrpnAddress(), 80);
    ugvVrpn = new MetaVrpnObject(name);

    getFrameworkManager()->AddDeviceToLog(ugvVrpn);
    vrpnclient->Start();

    // ========== UI: Boutons ==========
    //std::cout << "création du chemin suivi par le robot\n";
    Tab *ugvTab = new Tab(getFrameworkManager()->GetTabWidget(), "ugv", 0);
    GridLayout* buttonslayout = new GridLayout(ugvTab->NewRow(), "buttons");
    quitProgram = new PushButton(buttonslayout->NewRow(), "quit program");
    startGoal = new PushButton(buttonslayout->NewRow(), "start_goal");
    stopGoal = new PushButton(buttonslayout->LastRowLastCol(), "stop_goal");
    startLog = new PushButton(buttonslayout->NewRow(), "start_log");
    stopLog = new PushButton(buttonslayout->LastRowLastCol(), "stop_log");

    // ========== Création du planificateur DWA ==========
    /*trajectory = new dwa2Dtrajectory(vrpnclient->GetLayout()->NewRow(), 
                                     "Kinematic", 
                                     "DWA Costs");*/
    trajectory = new dwa2Dtrajectory(ugvTab->NewRow(), 
                                     "Kinematic", 
                                     "DWA Costs");
    std::cout << "création du chemin suivi par le robot\n";
    // ========== Configuration initiale des obstacles ==========
    trajectory->ClearObstacles();
    trajectory->AddObstacle(1.5f, 1.5f, 0.3f);
    trajectory->AddObstacle(3.0f, 3.5f, 0.4f);
    trajectory->AddObstacle(0.8f, 4.2f, 0.25f);
    
    std::cerr << "[dwa_path] Initial obstacles configured\n";

    // ========== Configuration initiale du goal ==========
    Vector2Df initial_goal(5.0f, 5.0f);
    trajectory->SetEnd(initial_goal);
    std::cerr << "[dwa_path] Initial goal set to (" << initial_goal.x 
              << ", " << initial_goal.y << ")\n";

    // ========== Ajout des courbes de trajectoire aux graphes ==========
    ugvVrpn->xPlot()->AddCurve(trajectory->GetMatrix()->Element(0, 0), DataPlot::Blue);
    ugvVrpn->yPlot()->AddCurve(trajectory->GetMatrix()->Element(0, 1), DataPlot::Blue);
    ugvVrpn->VxPlot()->AddCurve(trajectory->GetMatrix()->Element(1, 0), DataPlot::Blue);
    ugvVrpn->VyPlot()->AddCurve(trajectory->GetMatrix()->Element(1, 1), DataPlot::Blue);
    ugvVrpn->XyPlot()->AddCurve(trajectory->GetMatrix()->Element(0, 1),
                                trajectory->GetMatrix()->Element(0, 0),
                                DataPlot::Blue, "dwa_trajectory");

    // ========== Contrôleurs PID ==========
    Tab *lawTab = new Tab(getFrameworkManager()->GetTabWidget(), "control laws");
    GridLayout *lawLayout = new GridLayout(lawTab->NewRow(), "law_layout");
    TabWidget *tabWidget = new TabWidget(lawLayout->NewRow(), "laws");
    Tab *setupLawTab = new Tab(tabWidget, "Setup");
    Tab *graphLawTab = new Tab(tabWidget, "Graphes");
    
    uX = new Pid(setupLawTab->At(1, 0), "u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY = new Pid(setupLawTab->At(1, 1), "u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());

    getFrameworkManager()->AddDeviceToLog(uX);
    getFrameworkManager()->AddDeviceToLog(uY);

    // Paramètre L (empattement du robot)
    l = new DoubleSpinBox(setupLawTab->NewRow(), "L", " m", 0.1, 10, 0.1, 1, 1);

    std::cerr << "[dwa_path] Initialization complete\n";
}

// ========== DESTRUCTEUR ==========
dwa_path::~dwa_path() {
    // Le FrameworkManager gère la destruction des objets UI
}

// ========== BOUCLE PRINCIPALE ==========
void dwa_path::Run(void) {
    WarnUponSwitches(true);
    SetPeriodMS(20);  // 50 Hz

    if (getFrameworkManager()->ErrorOccured() == true) {
        SafeStop();
    }

    while (!ToBeStopped()) {
        SecurityCheck();
        CheckJoystick();
        CheckPushButton();

        if (behaviourMode == BehaviourMode_t::Manual) {
            ComputeManualControls();
        } else if (behaviourMode == BehaviourMode_t::GotoGoal) {
            ComputeGotoGoalControls();
        }
        
        WaitPeriod();
    }
}

// ========== VÉRIFICATION DES BOUTONS ==========
void dwa_path::CheckPushButton(void) {
    if (startLog->Clicked() == true) {
        getFrameworkManager()->StartLog();
    }
    if (stopLog->Clicked() == true) {
        getFrameworkManager()->StopLog();
    }

    if (startGoal->Clicked() == true) {
        std::cout << "[dwa_path] StartDriving called\n";
        StartDriving();
    }

    if (stopGoal->Clicked() == true) {
        std::cout << "[dwa_path] StopDriving called\n";
        StopDriving();
    }

    if (quitProgram->Clicked() == true) {
        SafeStop();
    }
}

// ========== VÉRIFICATION JOYSTICK ==========
void dwa_path::CheckJoystick(void) {
    // R1 + start => démarre le goal
    if (controller->ButtonClicked(4) && controller->IsButtonPressed(9)) {
        StartDriving();
    }
    // R1 + cross => arrête
    if (controller->ButtonClicked(5) && controller->IsButtonPressed(9)) {
        StopDriving();
    }
}

// ========== SÉCURITÉ VRPN ==========
void dwa_path::SecurityCheck(void) {
    if (behaviourMode == BehaviourMode_t::GotoGoal) {
        // Vérifie que le robot est toujours tracké
        if (!ugvVrpn->IsTracked(500)) {
            if (!vrpnLost) {
                Thread::Err("VRPN: UGV lost\n");
                vrpnLost = true;
                StopDriving();
            }
        } else {
            // VRPN récupéré
            if (vrpnLost) {
                Thread::Info("VRPN: UGV recovered\n");
                vrpnLost = false;
            }
        }
    }
}

// ========== CONTRÔLE MANUEL ==========
void dwa_path::ComputeManualControls(void) {
    float speed = -controller->GetAxisValue(3);
    float turn = controller->GetAxisValue(0);
    GetUgv()->GetUgvControls()->SetControls(speed, turn);
}

// ========== DÉMARRAGE DE LA NAVIGATION ==========
void dwa_path::StartDriving(void) {
    if (behaviourMode != BehaviourMode_t::GotoGoal) {
        // Vérification VRPN
        if (!ugvVrpn->IsTracked(100)) {
            Thread::Err("Cannot start: UGV VRPN not tracked\n");
            return;
        }

        // Récupération position actuelle
        Vector3Df ugv_pos3;
        Vector2Df ugv_2Dpos;
        ugvVrpn->GetPosition(ugv_pos3);
        ugv_pos3.To2Dxy(ugv_2Dpos);

        // Configuration du goal (peut être modifié dynamiquement)
        Vector2Df goal(5.0f, 5.0f);
        trajectory->SetEnd(goal);

        // Mise à jour des obstacles (optionnel)
        trajectory->ClearObstacles();
        trajectory->AddObstacle(2.0f, 2.0f, 0.3f);
        trajectory->AddObstacle(3.5f, 3.5f, 0.4f);

        // Démarrage de la trajectoire
        trajectory->StartTraj(ugv_2Dpos);

        // Réinitialisation
        vrpnLost = false;
        uX->Reset();
        uY->Reset();

        // Changement de mode
        behaviourMode = BehaviourMode_t::GotoGoal;

        std::cout << "[dwa_path] Started DWA from (" << ugv_2Dpos.x << ", " 
                  << ugv_2Dpos.y << ") toward goal (" << goal.x << ", " 
                  << goal.y << ")\n";
    }
}

// ========== ARRÊT DE LA NAVIGATION ==========
void dwa_path::StopDriving(void) {
    if (behaviourMode == BehaviourMode_t::GotoGoal) {
        trajectory->FinishTraj();
        behaviourMode = BehaviourMode_t::Manual;
        GetUgv()->GetUgvControls()->SetControls(0, 0);
        std::cout << "[dwa_path] Navigation stopped\n";
    }
}

// ========== CONTRÔLE AVEC DWA ==========
void dwa_path::ComputeGotoGoalControls(void) {
    // Paramètres de contrôle
    const float epsilon_pos = 0.05f;      // Seuil position : 5 cm
    const float epsilon_vel = 0.01f;      // Seuil vitesse : 1 cm/s
    const float alpha_filter = 0.3f;      // Coefficient filtre passe-bas

    Vector3Df ugv_pos3, ugv_vel3;
    Vector2Df ugv_2Dpos, ugv_2Dvel;
    Vector2Df pos_error, vel_error;
    Vector2Df traj_pos, traj_vel;

    // ========== LECTURE DES CAPTEURS ==========
    ugvVrpn->GetPosition(ugv_pos3);
    ugvVrpn->GetSpeed(ugv_vel3);
    ugv_pos3.To2Dxy(ugv_2Dpos);
    ugv_vel3.To2Dxy(ugv_2Dvel);

    // ========== MISE À JOUR DWA ==========
    trajectory->Update(GetTime());
    
    // Récupération du waypoint calculé par DWA
    trajectory->GetPosition(traj_pos);
    trajectory->GetSpeed(traj_vel);

    // ========== VÉRIFICATION D'ARRIVÉE ==========
    if (!trajectory->IsRunning()) {
        std::cout << "[dwa_path] Goal reached! Stopping.\n";
        GetUgv()->GetUgvControls()->SetControls(0, 0);
        behaviourMode = BehaviourMode_t::Manual;
        return;
    }

    // ========== CALCUL DES ERREURS ==========
    // CORRECTION :
    pos_error = traj_pos - ugv_2Dpos; // Consigne - Mesure
    vel_error = traj_vel - ugv_2Dvel; // Vitesse Consigne - Vitesse Mesure

    // Debug
    std::cout << "Robot: (" << ugv_2Dpos.x << ", " << ugv_2Dpos.y << ") "
              << "DWA target: (" << traj_pos.x << ", " << traj_pos.y << ")\n";

    // Reset PID si erreur négligeable
    if (pos_error.GetNorm() < epsilon_pos && vel_error.GetNorm() < epsilon_vel) {
        uX->Reset();
        uY->Reset();
    }

    // ========== CONTRÔLE PID ==========
    uX->SetValues(pos_error.x, vel_error.x);
    uX->Update(GetTime());
    uY->SetValues(pos_error.y, vel_error.y);
    uY->Update(GetTime());

    float u_x = uX->Output();
    float u_y = uY->Output();

    // ========== TRANSFORMATION REPÈRE MONDE → ROBOT ==========
    Quaternion vrpnQuaternion;
    ugvVrpn->GetQuaternion(vrpnQuaternion);
    float yaw = vrpnQuaternion.ToEuler().yaw;

    float Lval = l->Value();
    if (Lval < 1e-3f) Lval = 1e-3f;

    float vx_command = cosf(yaw) * u_x + sinf(yaw) * u_y;
    float wz_command = (-sinf(yaw) * u_x + cosf(yaw) * u_y) / Lval;

    // ========== FILTRE PASSE-BAS ==========
    static float vx_filtered = 0.0f;
    static float wz_filtered = 0.0f;
    vx_filtered = alpha_filter * vx_command + (1.0f - alpha_filter) * vx_filtered;
    wz_filtered = alpha_filter * wz_command + (1.0f - alpha_filter) * wz_filtered;

    // ========== ZONE MORTE ==========
    if (fabsf(vx_filtered) < epsilon_vel) vx_filtered = 0.0f;
    if (fabsf(wz_filtered) < epsilon_vel) wz_filtered = 0.0f;

    // ========== ENVOI DES COMMANDES ==========
    
    GetUgv()->GetUgvControls()->SetControls(-5, -2);

    // Debug
    //std::cerr << "Commands: vx=" << vx_filtered << " wz=" << wz_filtered << "\n";
}