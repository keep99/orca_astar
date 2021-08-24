#include <iostream>
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string_view>

#include <RVOSimulator.h>
#include "SDL2/SDL.h"
#include "SDL2/SDL_render.h"
#include <imgui.h>
#include <imgui_impl_sdl.h>
#include <imgui_sdl.h>

#include "astar.h"
#include "blockallocator.h"

using namespace std;

#if __EMSCRIPTEN__
#include <emscripten.h>
#endif
/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
struct Simulation
{
  enum configuration_t
  {
    CIRCLE,
    DEADLOCK,
    ASTAR,

    _CONFIGURATION_COUNT,
  };
  static constexpr std::array<std::string_view, _CONFIGURATION_COUNT>
    configuration_strings{
      "Circle",
      "Deadlock",
      "ASTAR",
    };
  struct options_t
  {
    configuration_t configuration{ ASTAR }; // 两种模式，一个中 CIRCLE，一种是 DEADLOCK，一种是 ASTAR
    bool run_simulation{ false };  // 设置为 false，刚进去先不演示，按空格键开始演示
    bool show_goal{ false };
    bool show_velocity{ true };
    float time_scale{ 10.0f };
    float neighborDist{ 5.0f };  // 原来是15.0f
    int maxNeighbors{ 10 };
    float timeHorizon{ 10.0f };
    float timeHorizonObst{ 10.0f };
    float radius{ 0.5f };  // 这个是 Agent 的半径，原本是 1.5f
    float maxSpeed{ 5.0f }; // 这个是 Agent 的最大速度，原来是 10.0f
    int numAgents{ 10 };  // 一个场景中的 Agent 数量，只有 CIRCLE 用到
    float circleRadius{ 200 };
  };
  Simulation() = default;

  // 初始化
  std::vector<RVO::Vector2> initialize(const options_t& options)
  {
    // 创建一个 RVO::RVOSimulator
    simulator = std::make_unique<RVO::RVOSimulator>();
    // 设置 Agent 的默认属性
    simulator->setAgentDefaults(options.neighborDist,
                                options.maxNeighbors,
                                options.timeHorizon,
                                options.timeHorizonObst,
                                options.radius,
                                options.maxSpeed);

    // 全局设置障碍物
    // std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;
    // obstacle1.push_back(RVO::Vector2(5, 5));
    // obstacle1.push_back(RVO::Vector2(5, -5));
    // obstacle1.push_back(RVO::Vector2(-5, -5));
    // obstacle1.push_back(RVO::Vector2(-5, 5));
    // obstacles.push_back(obstacle1);


    // 将 obstacles 中的所有障碍加入到 simulator 中
    for (const auto& obstacle : obstacles) {
      simulator->addObstacle(obstacle);
    }
    if (!obstacles.empty()) {
      simulator->processObstacles();
    }

    // 往场景中加 Agent
    // 原场景就是 一个大圆圈
    // 默认是 250个 Agent
    // 目标点在圆的另外一边
    goals.clear();
    // 场景一：
    // 对应的是 CIRCLE 模式
    if (options.configuration == CIRCLE) {
      goals.reserve(options.numAgents);
      for (size_t i = 0; i < options.numAgents; ++i) {
        // addAgent 是指定这个 Agent 的位置
        simulator->addAgent(
          options.circleRadius *
          RVO::Vector2(std::cos(i * 2 * M_PI / options.numAgents),
                       std::sin(i * 2 * M_PI / options.numAgents)));
        goals.push_back(-simulator->getAgentPosition(i));
      }

      // 通过增加 Agent 来设置障碍物
      // simulator->addAgent(RVO::Vector2(0, 0),
      //   0, 0, 0, 0, 80, 0, RVO::Vector2(0, 0));
      // goals.push_back(RVO::Vector2(0, 0));

    } 
    // 场景二：
    // 对应的是 DEADLOCK 模式
    else if (options.configuration == DEADLOCK) {
      // Phalanx moving right
      // 情况一：
      // 设置两排的 Agent 方阵
      for (uint8_t i = 0; i < 2; ++i) {
        for (uint8_t j = 0; j < 12; ++j) {
          simulator->addAgent(RVO::Vector2((-5 - 2 * i) * options.radius,
                                           (j * 2) * options.radius));
          goals.emplace_back((40 - 2 * i) * options.radius, (j * 2) * options.radius);
          if (j > 0) {
            simulator->addAgent(RVO::Vector2((-5 - 2 * i) * options.radius,
                                             (j * -2) * options.radius));
            goals.emplace_back((40 - 2 * i) * options.radius, (j * -2) * options.radius);
          }
        }
      }
      // Single agent moving left trying get passed Phalanx
      // 单独的一个 Agent 试图穿过这个方阵
      simulator->addAgent(RVO::Vector2(5 * options.radius, 0));
      goals.emplace_back(-40 * options.radius, 0);

      // 情况二：
      // 两个共线的 Agent
      // Two collinear agents
      simulator->addAgent(RVO::Vector2((-2 - 20) * options.radius, -10));
      goals.emplace_back((10 - 20) * options.radius, -10);
      simulator->addAgent(RVO::Vector2((2 - 20) * options.radius, -10));
      goals.emplace_back((-10 - 20) * options.radius, -10);
      // 一个单独的 Agent 下来穿过这两个共线的 Agent
      // Agent going down who will disrupt the collinear deadlock
      simulator->addAgent(RVO::Vector2(-20 * options.radius, 100));
      goals.emplace_back(-20 * options.radius, -100);
    }
    
    // 场景三：
    else if(options.configuration == ASTAR) {
      // 定义了 Agent 的初始位置和目标位置
      simulator->addAgent(RVO::Vector2(5, 5));
      goals.emplace_back(5, 5);  // 暂时先让他不动，不加的话不行
      // 为了演示 ORCA 添加的智能体
      simulator->addAgent(RVO::Vector2(29, 200));
      goals.emplace_back(29, -100);

      /* 地图信息 */
      // char maps[10][10] = 
      // {
      //   {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
      //   {0, 1, 1, 1, 0, 1, 0, 1, 0, 1},
      //   {1, 1, 1, 1, 0, 1, 0, 1, 0, 1},
      //   {0, 0, 0, 1, 0, 1, 0, 1, 0, 1},
      //   {0, 1, 0, 1, 0, 1, 1, 1, 0, 1},
      //   {0, 1, 0, 1, 0, 1, 0, 0, 0, 1},
      //   {0, 1, 1, 1, 0, 1, 1, 1, 1, 1},
      //   {0, 0, 0, 1, 0, 1, 0, 0, 1, 0},
      //   {1, 1, 0, 1, 0, 1, 0, 0, 0, 0},
      //   {0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
      // };
      
      char maps[61][61];
      for(int i = 0; i <= 60; ++i) {
        for(int j = 0; j <= 60; ++j) {
          maps[i][j] = '0';
        }
      }

      // maps 的障碍物
      // for(int i = 0; i <= 25; ++i) {
      //   for(int j = 15; j <= 60; ++j) {
      //     maps[i][j] = '1';
      //   }
      // }
      for(int i = 15; i <= 60; ++i) {
        for(int j = 0; j <= 25; ++j) {
          maps[i][j] = '1';
        }
      }

      // for(int i = 30; i <= 60; ++i) {
      //   for(int j = 0; j <= 50; ++j) {
      //     maps[i][j] = '1';
      //   }
      // }

      for(int i = 0; i <= 50; ++i) {
        for(int j = 30; j <= 60; ++j) {
          maps[i][j] = '1';
        }
      }

      for(int i = 0; i <= 60; ++i) {
        for(int j = 0; j <= 60; ++j) {
          cout << maps[i][j];
        }
        cout << endl;
      }


      // 添加障碍物
      // 两个障碍物
      std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;
      obstacle1.push_back(RVO::Vector2(0, 15));
      obstacle1.push_back(RVO::Vector2(25, 15));
      obstacle1.push_back(RVO::Vector2(25, 60));
      obstacle1.push_back(RVO::Vector2(0, 60));
      obstacles.push_back(obstacle1);

      obstacle2.push_back(RVO::Vector2(35, 0));
      obstacle2.push_back(RVO::Vector2(60, 0));
      obstacle2.push_back(RVO::Vector2(60, 50));
      obstacle2.push_back(RVO::Vector2(35, 50));
      obstacles.push_back(obstacle2);

      // Astart
      // 搜索参数
      AStar::Params param;
      param.width = 61;
      param.height = 61;
      param.corner = false;
      param.start = AStar::Vec2(5, 5);
      param.end = AStar::Vec2(55, 55);
      param.can_pass = [&](const AStar::Vec2 &pos) -> bool
      {
          return maps[pos.y][pos.x] == '0';
      };

      // 执行搜索
      BlockAllocator allocator;
      AStar algorithm(&allocator);
      // 得到每一步的 goal
      std::vector<AStar::Vec2> path = algorithm.find(param);
      if(path.size() == 0) {
        cout << "???????????????????????????????????????????" << endl;
      }
      for(int i = 0; i < path.size(); ++i) {
        cout << "ppppppppppppppppppppppppp" << endl;
        cout << path[i].x << " " << path[i].y << endl;
      }

      // TO DO
      // 优化一：
      // 进行路径的合并
      // param.start = AStar::Vec2(0, 0);
      // param.end = AStar::Vec2(9, 9);
      // std::vector<RVO::Vector2> CompressedPath;
      // for(int i = 0; i < path.size(); ++i) {
      //   CompressedPath.push_back(RVO::Vector2(path[i].x, path[i].y));
      // }
      
      // for(int i = 0; i < path.size(); ++i) {
      //   cout << path[i].x << " " << path[i].y << endl;
      // }
      
      // return CompressedPath;

      // 先 return 
      std::vector<RVO::Vector2> tmp;
      // 起点
      tmp.push_back(RVO::Vector2(5,5));
      // 剩余的路径点
      // tmp.push_back(RVO::Vector2(30,5));
      // tmp.push_back(RVO::Vector2(30,55));
      // tmp.push_back(RVO::Vector2(55,55));
      for(int i = 0; i < path.size(); ++i) {
        tmp.push_back(RVO::Vector2(path[i].x, path[i].y));
      }

      return tmp;
      // 定义障碍物的位置
      // std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;
      // obstacle1.push_back(RVO::Vector2(5, 5));
      // obstacle1.push_back(RVO::Vector2(5, -5));
      // obstacle1.push_back(RVO::Vector2(-5, -5));
      // obstacle1.push_back(RVO::Vector2(-5, 5));
      // obstacles.push_back(obstacle1);
      
    }

    // // 对场景中的所有 Agent 设置 偏好速度
    // set_preferred_velocities();
  }

  void set_preferred_velocities()
  {
    for (int i = 0; i < static_cast<int>(simulator->getNumAgents()); ++i) {
      RVO::Vector2 goalVector = goals[i] - simulator->getAgentPosition(i);

      // i 是 Agent 的编号，goalVector 是偏好速度，也就是 目标的位置减去当前的位置所得的向量
      simulator->setAgentPrefVelocity(i, goalVector);
    }
  }

  void step(float dt)
  {
    simulator->setTimeStep(dt);
    // 只是做了一步，所以后面自动跳到了 (50, 50)
    // TO DO：如果 当前的 Agent 没有到达 最终的目标位置，持续地做 doStep
    simulator->doStep();
    // do {
    //   cout << "qqqqqqqqqqqqqq" << endl;
    //   simulator->doStep();
    // } while(simulator->getAgentPosition(1) != goals[0]);

  }

  void commit_obstacle()
  {
    if (staging_obstacle.size() > 2) {
      simulator->addObstacle(staging_obstacle);
      simulator->processObstacles();
      obstacles.emplace_back(staging_obstacle);
    }
    staging_obstacle.clear();
  }

  std::unique_ptr<RVO::RVOSimulator> simulator;
  std::vector<RVO::Vector2> goals;
  std::vector<RVO::Vector2> staging_obstacle;
  std::vector<std::vector<RVO::Vector2>> obstacles;
};

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
// 和图像的显示相关
struct Renderer
{
  struct options_t
  {
    float scale{ 1.5f };
    float offset_x{ 300 };
    float offset_y{ 0 };
    // 背景颜色
    // 目标颜色， 黑色
    // 速度颜色
    uint8_t background_color[3] = { 0x37, 0x47, 0x4F };  // { 0x37, 0x47, 0x4F }
    uint8_t goal_color[3] = { 0x00, 0x00, 0x00 };
    uint8_t velocity_color[3] = { 0x23, 0xC7, 0xAC };
  };

public:
  Renderer()
    : window(nullptr)
    , renderer(nullptr)
    , ui(nullptr)
  {}
  ~Renderer() { terminate(); }

  bool initialize()
  {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
      std::printf("Failed to initialize SDL\n");
      return false;
    }

    // 窗口标题
    window = SDL_CreateWindow(
      "Collision Avoidance", SDL_HINT_DEFAULT, SDL_HINT_DEFAULT, 1280, 768, 0);
    if (!window) {
      std::printf("Failed to create SDL window\n");
      terminate();
      return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
      std::printf("Failed to create SDL renderer\n");
      terminate();
      return false;
    }

    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    width = w;
    height = h;

    IMGUI_CHECKVERSION();
    ui = ImGui::CreateContext();
    if (!ui) {
      std::printf("Failed to create ImGui context\n");
      terminate();
      return false;
    }

    ImGuiSDL::Initialize(renderer, width, height);
    ImGui_ImplSDL2_InitForOpenGL(window, nullptr);

    return true;
  }

  void terminate() noexcept
  {
    if (ui) {
      ImGuiSDL::Deinitialize();
      ImGui_ImplSDL2_Shutdown();
      ImGui::DestroyContext(ui);
      ui = nullptr;
    }

    if (renderer) {
      SDL_DestroyRenderer(renderer);
      renderer = nullptr;
    }
    if (window) {
      SDL_DestroyWindow(window);
      window = nullptr;
    }
    SDL_Quit();
  }

  RVO::Vector2 toScreenSpace(const RVO::Vector2& point)
  {
    return {
      width / 2 + options.offset_x + point.x() * options.scale,
      height / 2 + options.offset_y - point.y() * options.scale,
    };
  }

  RVO::Vector2 fromScreenSpace(const RVO::Vector2& point)
  {
    return RVO::Vector2{
      point.x() - width / 2 - options.offset_x,
      -point.y() + height / 2 + options.offset_y,
    } / options.scale;
  }

  // 画
  void draw(float dt,
            Simulation& simulation,
            Simulation::options_t& simulation_options)
  {
    ImGui_ImplSDL2_NewFrame(window);
    ImGui::NewFrame();

    // 标题
    ImGui::Begin("Controls");
    // 添加一些字段
    ImGui::Text("dt: %.5f seconds", dt);
    ImGui::Text("Keyboard controls:\n"
                "\tSpacebar: Pause/Continue Simulation.\n"
                "\tBackspace: Reset Simulation.\n"
                "Mouse controls:\n"
                "\tDouble click to add obstacle vertex\n"
                "\tRight click to finish obstacle");

    // 以下添加一些悬浮的选项
    ImGui::SliderFloat("Zoom", &options.scale, 0.01, 100, "%.3f", 2.0f);
    float offset_max =
      static_cast<float>(std::max(width, height)) * 0.5f +
      (simulation_options.circleRadius + simulation_options.radius) *
        options.scale;
    ImGui::SliderFloat2(
      "Offset", &options.offset_x, -offset_max, offset_max + 1);
    ImGui::SliderFloat(
      "Time Scale", &simulation_options.time_scale, 0.01, 100, "%.3f", 2.0f);
    ImGui::SliderFloat(
      "Neighbor Distance (m)", &simulation_options.neighborDist, 0, 50);
    ImGui::SliderInt("Max Neighbors", &simulation_options.maxNeighbors, 0, 50);
    ImGui::SliderFloat(
      "Tau for other agents (s)", &simulation_options.timeHorizon, 0, 50);
    ImGui::SliderFloat(
      "Tau for Obstacles (s)", &simulation_options.timeHorizonObst, 0, 50);
    ImGui::SliderFloat("Agent Radius (m)", &simulation_options.radius, 0, 10);
    ImGui::SliderFloat(
      "Agent Max Speed (m/s)", &simulation_options.maxSpeed, 0, 100);
    ImGui::SliderInt("Number of Agents", &simulation_options.numAgents, 0, 500);
    ImGui::SliderFloat(
      "Radius of Circle (m)", &simulation_options.circleRadius, 0, 1000);

    ImGui::Checkbox("Show Goal", &simulation_options.show_goal);
    ImGui::Checkbox("Show Preferred velocity",
                    &simulation_options.show_velocity);
    ImGui::Checkbox("Run Simulation", &simulation_options.run_simulation);

    auto item_current =
      Simulation::configuration_strings[simulation_options.configuration];
    if (ImGui::BeginCombo("Configuration", item_current.data())) {
      for (int n = 0; n < Simulation::configuration_strings.size(); n++) {
        bool is_selected =
          (item_current == Simulation::configuration_strings[n]);
        if (ImGui::Selectable(Simulation::configuration_strings[n].data(),
                              is_selected)) {
          simulation_options.configuration =
            static_cast<Simulation::configuration_t>(n);
        }
        if (is_selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }

    if (!simulation.staging_obstacle.empty()) {
      if (ImGui::Button("Add Obstacle")) {
        simulation.commit_obstacle();
      }
    }

    // 重置选项，但是有误
    if (ImGui::Button("Reset")) {
      simulation.initialize(simulation_options);
    }
    ImGui::End();

    const SDL_Rect clip = {
      0, 0, static_cast<int>(width), static_cast<int>(height)
    };
    SDL_RenderSetClipRect(renderer, &clip);

    SDL_SetRenderDrawColor(renderer,
                           options.background_color[0],
                           options.background_color[1],
                           options.background_color[2],
                           SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    if (simulation_options.show_goal) {
      SDL_SetRenderDrawColor(renderer,
                             options.goal_color[0],
                             options.goal_color[1],
                             options.goal_color[2],
                             SDL_ALPHA_OPAQUE);
      for (uint32_t i = 0; i < simulation.simulator->getNumAgents(); ++i) {
        auto point = toScreenSpace(simulation.simulator->getAgentPosition(i));
        auto goal = toScreenSpace(simulation.goals[i]);
        SDL_RenderDrawLine(renderer, point.x(), point.y(), goal.x(), goal.y());
      }
    }

    if (simulation_options.show_velocity) {
      SDL_SetRenderDrawColor(renderer,
                             options.velocity_color[0],
                             options.velocity_color[1],
                             options.velocity_color[2],
                             SDL_ALPHA_OPAQUE);
      for (uint32_t i = 0; i < simulation.simulator->getNumAgents(); ++i) {
        auto point = toScreenSpace(simulation.simulator->getAgentPosition(i));
        auto pref_velocity = simulation.simulator->getAgentPrefVelocity(i);
        auto max_speed = simulation.simulator->getAgentMaxSpeed(i);
        auto goal = simulation.simulator->getAgentPosition(i);
        if (RVO::absSq(pref_velocity) > max_speed * max_speed) {
          goal += RVO::normalize(pref_velocity) * max_speed;
        } else {
          goal += pref_velocity;
        }
        goal = toScreenSpace(goal);
        SDL_RenderDrawLine(renderer, point.x(), point.y(), goal.x(), goal.y());
      }
    }

    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE);
    for (uint32_t i = 0; i < simulation.simulator->getNumAgents(); ++i) {
      auto point = toScreenSpace(simulation.simulator->getAgentPosition(i));
      int w = static_cast<int>(simulation_options.radius * 2 * options.scale);
      int h = static_cast<int>(simulation_options.radius * 2 * options.scale);
      if (w > 1 && h > 1) {
        SDL_Rect rect{
          static_cast<int>(point.x() -
                           simulation_options.radius * options.scale),
          static_cast<int>(point.y() -
                           simulation_options.radius * options.scale),
          w,
          h,
        };
        SDL_RenderDrawRect(renderer, &rect);
      } else {
        SDL_RenderDrawPoint(renderer, point.x(), point.y());
      }
    }

    // 障碍物
    SDL_SetRenderDrawColor(renderer, 0x7F, 0x7F, 0x7F, SDL_ALPHA_OPAQUE);
    RVO::Vector2 previous;
    for (const auto& obstacle : simulation.obstacles) {
      if (obstacle.size() < 3) {
        continue;
      }
      previous = toScreenSpace(obstacle.back());
      for (auto i : obstacle) {
        auto point = toScreenSpace(i);
        SDL_RenderDrawLine(
          renderer, previous.x(), previous.y(), point.x(), point.y());
        previous = point;
      }
    }

    SDL_SetRenderDrawColor(renderer, 0x7F, 0x7F, 0x7F, SDL_ALPHA_OPAQUE);
    for (uint32_t i = 0; i < simulation.staging_obstacle.size(); ++i) {
      auto point = toScreenSpace(simulation.staging_obstacle[i]);
      SDL_RenderDrawPoint(renderer, point.x(), point.y());
      if (i > 0) {
        SDL_RenderDrawLine(
          renderer, previous.x(), previous.y(), point.x(), point.y());
      }
      previous = point;
    }
    if (ui_want_capture_mouse()) {
      if (simulation.staging_obstacle.size() > 2) {
        auto point_1 = toScreenSpace(simulation.staging_obstacle[0]);
        auto point_2 = toScreenSpace(simulation.staging_obstacle.back());
        SDL_RenderDrawLine(
          renderer, point_1.x(), point_1.y(), point_2.x(), point_2.y());
      }
    } else {
      if (!simulation.staging_obstacle.empty()) {
        int x, y;
        SDL_GetMouseState(&x, &y);
        auto point = toScreenSpace(simulation.staging_obstacle[0]);
        SDL_RenderDrawLine(renderer, point.x(), point.y(), x, y);
      }
      if (simulation.staging_obstacle.size() > 1) {
        int x, y;
        SDL_GetMouseState(&x, &y);
        auto point = toScreenSpace(simulation.staging_obstacle.back());
        SDL_RenderDrawLine(renderer, point.x(), point.y(), x, y);
      }
    }

    ImGui::Render();
    ImGuiSDL::Render(ImGui::GetDrawData());

    SDL_RenderPresent(renderer);
  }

  void resolution(uint32_t& out_width, uint32_t& out_height) const
  {
    out_width = width;
    out_height = height;
  }

  bool ui_want_capture_mouse() const
  {
    ImGuiIO& io = ImGui::GetIO();
    return io.WantCaptureMouse;
  }

  bool ui_want_capture_keyboard() const
  {
    ImGuiIO& io = ImGui::GetIO();
    return io.WantCaptureKeyboard;
  }
  // SDL_Window：结构体，描述了一个窗体对象，表示的是会呈现在设备上一个窗体，所有图像的载体。
  // 一般通过SDL_CreateWindow函数创建，和SDL_DestroyWindow销毁
  SDL_Window* window;
  // 渲染器
  SDL_Renderer* renderer;
  ImGuiContext* ui;
  options_t options;
  uint32_t width;
  uint32_t height;
};

#if __EMSCRIPTEN__
void
em_main_loop_callback(void* arg);
#endif

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
class App
{
public:
  // 构造函数
  App()
    : time_stamp(std::chrono::high_resolution_clock::now())
  {
    path = simulation.initialize(simulation_options);
  }
  virtual ~App() = default;

  bool main_loop(int& i)
  {
    // cout << simulation.goals[0].x() << "iii" << simulation.goals[0].y() << endl;
    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<float>>(
      now - time_stamp);
    time_stamp = now;

    // 每一次 main_loop 的执行，都要 draw
    renderer.draw(dt.count(), simulation, simulation_options);

    cout << "当前位置："
         << simulation.simulator->getAgentPosition(0).x() << " "
         << simulation.simulator->getAgentPosition(0).y() << endl;

    // cout << "当前的目标位置："
    //      << path[i].x() << " "
    //      << path[i].y() << endl;
    
    // 可能的 Bug 是 float 的相等性比较
    // if(simulation.simulator->getAgentPosition(0) == path[i] && ++i < path.size()) { // 或者等于 simulation.goals[0]
    if(abs(simulation.simulator->getAgentPosition(0).x() - path[i].x()) < 10e-4 &&
       abs(simulation.simulator->getAgentPosition(0).y() - path[i].y()) < 10e-4 &&
       ++i < path.size()) {
      // cout << "xxxxxxxxx" << endl;
      simulation.goals[0].x_ = path[i].x();
      simulation.goals[0].y_ = path[i].y();
    }

    if (simulation_options.run_simulation) {
      simulation.set_preferred_velocities();
      
      // 以下是 更新 Agent0 的逻辑
      RVO::Vector2 goalVector = simulation.goals[0] - simulation.simulator->getAgentPosition(0);
      // i 是 Agent 的编号，goalVector 是偏好速度，也就是 目标的位置减去当前的位置所得的向量
      simulation.simulator->setAgentPrefVelocity(0, goalVector * 10);

      simulation.step(simulation_options.time_scale * dt.count());
    }

    SDL_Event event;
    // 这边是在SDL 在监控用户的行为
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      uint32_t width, height;
      renderer.resolution(width, height);
      // 退出
      if (event.type == SDL_QUIT) {
        return false;
      } 
      // 鼠标点击
      else if (event.type == SDL_MOUSEBUTTONDOWN &&
                 !renderer.ui_want_capture_mouse()) {
        if (event.button.button == SDL_BUTTON_LEFT &&
            event.button.clicks == 2) {
          // simulation.staging_obstacle.emplace_back(renderer.fromScreenSpace(
          //   RVO::Vector2(event.button.x, event.button.y)));
            simulation.staging_obstacle.emplace_back(renderer.fromScreenSpace(
            RVO::Vector2(event.button.x, event.button.y)));
        } else if (event.button.button == SDL_BUTTON_RIGHT) {
          simulation.staging_obstacle.emplace_back(renderer.fromScreenSpace(
            RVO::Vector2(event.button.x, event.button.y)));
          simulation.commit_obstacle();
        }
      } 
      // 鼠标轮
      else if (event.type == SDL_MOUSEWHEEL &&
                 !renderer.ui_want_capture_mouse()) {
        renderer.options.scale += event.wheel.y * 0.01 * renderer.options.scale;
        if (renderer.options.scale < 0) {
          renderer.options.scale = 0.0f;
        }
      } 
      // 鼠标移动
      else if (event.type == SDL_MOUSEMOTION &&
                 event.motion.state == SDL_PRESSED &&
                 !renderer.ui_want_capture_mouse()) {
        renderer.options.offset_x += event.motion.xrel;
        renderer.options.offset_y += event.motion.yrel;
      } 
      // 键盘相关操作
      else if (event.type == SDL_KEYDOWN &&
                 !renderer.ui_want_capture_keyboard()) {
        switch (event.key.keysym.scancode) {
          case SDL_SCANCODE_ESCAPE:
            return false;
          // 按空格键开始运行
          // 这边的一个 Bug 是，当前设置为 (25,0)之后，执行到这边
          // 设置了 simulation_options.run_simulation 为 true
          // 但是 break 掉了，进入了 while(main_loop 循环体内)，导致对 goals[0] 重新赋值为 (50,50)
          case SDL_SCANCODE_SPACE:
            // 按下空格键之后，将 simulation_options.run_simulation 置为 true
            simulation_options.run_simulation =
              !simulation_options.run_simulation;
            // 必须先执行一次，不然会直接换成下一个路径点
            // simulation.set_preferred_velocities();
            // simulation.step(simulation_options.time_scale * dt.count());
            break;
          case SDL_SCANCODE_BACKSPACE:
            simulation.initialize(simulation_options);
            break;
          // default:
          //   break;
        }
      }
    }
    return true;
  }

  int run()
  {
    // 先进行 renderer 的初始化
    if (!renderer.initialize()) {
      return EXIT_FAILURE;
    }

    uint32_t width, height;
    renderer.resolution(width, height);

#if __EMSCRIPTEN__
    emscripten_set_main_loop_arg(em_main_loop_callback, this, 0, true);
#else
    // 主循环
    int i = 0;
    while (main_loop(i)) {
    }
#endif

    terminate();

    return EXIT_SUCCESS;
  }

  void terminate() { renderer.terminate(); }

private:
  std::chrono::time_point<
#if __EMSCRIPTEN__
    std::chrono::steady_clock
#else
    std::chrono::system_clock
#endif
    >
    time_stamp;
  
  Simulation::options_t simulation_options;
public:
  Renderer renderer;
  std::vector<RVO::Vector2> path;  // 存放 Astar 的路径点
  Simulation simulation;
};

#if __EMSCRIPTEN__
void
em_main_loop_callback(void* arg)
{
  auto app = reinterpret_cast<App*>(arg);
  if (!app->main_loop()) {
    app->terminate();
    emscripten_cancel_main_loop();
  }
}
#endif


/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
int
main(int argc, char* argv[])
{
  cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << endl;
  App app;
  for(int i = 0; i < app.path.size(); ++i) {
    cout << app.path[i].x() << " " << app.path[i].y() << endl;
  }

  // if (!app.renderer.initialize()) {
  //     return EXIT_FAILURE;
  // }
  // uint32_t width, height;
  // app.renderer.resolution(width, height);
 
  // for(int i = 0; i < app.path.size(); ++i) {
  //   // 设置当前的目标位置
  //   app.simulation.goals[0].x_ = app.path[i].x();
  //   app.simulation.goals[0].y_ = app.path[i].y();
  //   app.simulation.set_preferred_velocities();

  // #if __EMSCRIPTEN__
  //   emscripten_set_main_loop_arg(em_main_loop_callback, this, 0, true);
  // #else
  //   // 主循环
  //   while (app.main_loop()) {
  //   }
  // #endif
  
  // }
  // int i = 0;
  // while(app.main_loop()) {
  //   if(i < app.path.size()) {
  //     // 设置 Agent 当前目标位置
  //     cout << app.path[i].x() << "iiii" << app.path[i].y() << endl;
  //     app.simulation.goals[0].x_ = app.path[i].x();
  //     app.simulation.goals[0].y_ = app.path[i].y();
  //   }

  //   i++;
  // }

  // app.simulation.goals[0].x_ = app.path[0].x();
  // app.simulation.goals[0].y_ = app.path[0].y();
  // app.main_loop();

  // terminate();

  // return EXIT_SUCCESS;

//   int run()
//   {
//     // 先进行 renderer 的初始化
//     if (!renderer.initialize()) {
//       return EXIT_FAILURE;
//     }

//     uint32_t width, height;
//     renderer.resolution(width, height);

// #if __EMSCRIPTEN__
//     emscripten_set_main_loop_arg(em_main_loop_callback, this, 0, true);
// #else
//     // 主循环
//     while (main_loop()) {
//     }
// #endif

//     terminate();

//     return EXIT_SUCCESS;
//   }


  return app.run();
}
