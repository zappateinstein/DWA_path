//  created:    2020/11/20
//  filename:   main.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 6599
//
//  version:    $Id: $
//
//  purpose:    main simulateur
//
//
/*********************************************************************/

#include <tclap/CmdLine.h>
#include <Simulator.h>
#include <TwoWheelRobot.h>
#ifdef GL
#include <Parser.h>
#include <Blade.h>
#include <Castle.h>
#include <DoubleSpinBox.h>
#include <Tab.h>
#include <ISceneManager.h>
#include <MeshSceneNode.h>
#include <random>
#include <vector>
#include <string>
#include <ctime>
#endif

using namespace TCLAP;
using namespace std;
using namespace flair::simulator;
using namespace flair::sensor;
#ifdef GL
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace flair::gui;
#endif

int port;
int opti_time;
std::string xml_file;
std::string media_path;
std::string scene_file;
std::string name;
std::string address;

// utilities are included above under GL

void parseOptions(int argc, char** argv)
{
  try {
    CmdLine cmd("Command description message", ' ', "0.1");

    ValueArg<std::string> nameArg("n", "name", "uav name, also used for vrpn", true, "x4", "string");
    cmd.add(nameArg);

    ValueArg<std::string> xmlArg("x", "xml", "xml file", true, "./reglages.xml", "string");
    cmd.add(xmlArg);

    ValueArg<int> portArg("p", "port", "ground station port", true, 9002, "int");
    cmd.add(portArg);

    ValueArg<std::string> addressArg("a", "address", "ground station address", true, "127.0.0.1", "string");
    cmd.add(addressArg);

    ValueArg<int> optiArg("o", "opti", "optitrack time ms", false, 0, "int");
    cmd.add(optiArg);

#ifdef GL
    ValueArg<std::string> mediaArg("m", "media", "path to media files", true, "./", "string");
    cmd.add(mediaArg);

    ValueArg<std::string> sceneArg("s", "scene", "path to scene file", true, "./voliere.xml", "string");
    cmd.add(sceneArg);
#endif

    cmd.parse(argc, argv);

    // Get the value parsed by each arg.
    port = portArg.getValue();
    xml_file = xmlArg.getValue();
    opti_time = optiArg.getValue();
    name = nameArg.getValue();
    address = addressArg.getValue();
#ifdef GL
    media_path = mediaArg.getValue();
    scene_file = sceneArg.getValue();
#endif

  } catch(ArgException& e) {
    cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
    exit(EXIT_FAILURE);
  }
}

// Générateur sûr d'obstacles : construit et renvoie des pointeurs `Ball*`
// sans appeler d'API spécifique (positionnement / enregistrement au simulateur).
#ifdef GL
class Ball : public Model {
public:
    Ball(std::string name, int id, float x, float y, float z) : Model(name, id), x(x), y(y), z(z) {
        Tab *setup_tab = GetParamsTab();
        radius = new DoubleSpinBox(setup_tab->NewRow(), "radius :", "m", 0, 10, 0.1, 2, 0.5);
        SetIsReady(true);
    }

    void Draw(void) {
        node = getGui()->getSceneManager()->addSphereSceneNode(100, 16, getSceneNode());
        
        ITexture *texture = getGui()->getTexture("ball.jpg");
        if(texture) node->setMaterialTexture(0, texture);
        node->setMaterialFlag(video::EMF_LIGHTING, false);

        setTriangleSelector(getGui()->getSceneManager()->createTriangleSelector(node->getMesh(), node));
        
        node->setScale(vector3df(radius->Value(), radius->Value(), radius->Value()));
        node->setPosition(vector3df(x, y, z));
    }
    
    void AnimateModel(void) {
        if (radius->ValueChanged() == true) {
            node->setScale(vector3df(radius->Value(), radius->Value(), radius->Value()));
        }
    }

    size_t dbtSize(void) const override { return 0; }
    void WritedbtBuf(char *dbtbuf) override {}
    void ReaddbtBuf(char *dbtbuf) override {}
    void CalcModel(void) override {}

private:
    irr::scene::IMeshSceneNode *node;
    DoubleSpinBox *radius;
    float x, y, z;
};

static std::vector<Ball*> generateObstaclesSafe(
    int count,
    float minX, float maxX,
    float minY, float maxY,
    float z = 0.0f)
{
  std::vector<Ball*> obs;
  std::mt19937 rng(static_cast<unsigned>(std::time(nullptr)));
  std::uniform_real_distribution<float> dx(minX, maxX), dy(minY, maxY);

  for (int i = 0; i < count; ++i) {
    float x = dx(rng);
    float y = dy(rng);

    // construit l'objet graphique/visuel ; ne modifie pas sa position
    Ball* o = new Ball(std::string("obs_") + std::to_string(i), /*id*/ i + 1, x, y, z);

    // Positionnement et ajout au simulateur doivent être faits avec
    // les méthodes réelles de l'API (à appeler après inspection des headers).
    obs.push_back(o);
  }
  return obs;
}
#endif

int main(int argc, char* argv[]) {
  Simulator* simu;
  Model* robot;
#ifdef GL
  Parser* gui;
#endif
  parseOptions(argc, argv);

  simu = new Simulator("simulator", opti_time, 90);
  simu->SetupConnection(address, port);
  simu->SetupUserInterface(xml_file);

#ifdef GL
  gui = new Parser(960, 480, 960, 480, media_path, scene_file);
  robot = new TwoWheelRobot(name, 0);
  
  // Generate obstacles
  std::vector<Ball*> obstacles = generateObstaclesSafe(5, -4.0f, 4.0f, -4.0f, 4.0f);

  gui->setVisualizationCamera(robot);
  simu->RunSimu();
#endif
  delete simu;

  return 0;
}

