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
    /* Add bu Chen.Yu. */
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
    configuration_t configuration{ CIRCLE }; // 两种模式，一个中 CIRCLE，一种是 DEADLOCK，一种是 ASTAR
    bool run_simulation{ false };  // 设置为 false，刚进去先不演示，按空格键开始演示
    bool show_goal{ true };
    bool show_velocity{ true };
    float time_scale{ 10.0f };
    float neighborDist{ 15.0f };  // 原来是15 这个是
    int maxNeighbors{ 10 };
    float timeHorizon{ 10.0f };
    float timeHorizonObst{ 10.0f };
    float radius{ 1.5f };  // 这个是 Agent 的半径
    float maxSpeed{ 10.0f };
    int numAgents{ 200 };  // 一个场景中的 Agent 数量，只有 CIRCLE 用到
    float circleRadius{ 200 };
  };
  Simulation() = default;

  // 初始化
  void initialize(const options_t& options)
  {
    // 创建一个 RVO::RVOSimulator
    simulator = std::make_unique<RVO::RVOSimulator>();
    /* Specify the default parameters for agents that are subsequently added. */
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
    /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */
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
    
    /* Add by Chen.Yu. */
    // 场景三：
    else if(options.configuration == ASTAR) {
      // 定义了 Agent 的初始位置和目标位置
      simulator->addAgent(RVO::Vector2(0, 0));
      goals.emplace_back(50, 50);  // 这个是最终的目的地
      // simulator->addAgent(RVO::Vector2(9, 9));
      // goals.emplace_back(0, 0);


      /* 地图信息 */
      char maps[10][10] = 
      {
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 1, 1, 1, 0, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 0, 1, 0, 1, 0, 1},
        {0, 0, 0, 1, 0, 1, 0, 1, 0, 1},
        {0, 1, 0, 1, 0, 1, 1, 1, 0, 1},
        {0, 1, 0, 1, 0, 1, 0, 0, 0, 1},
        {0, 1, 1, 1, 0, 1, 1, 1, 1, 1},
        {0, 0, 0, 1, 0, 1, 0, 0, 1, 0},
        {1, 1, 0, 1, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
      };

      // 添加障碍物
      std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;
      obstacle1.push_back(RVO::Vector2(10, 10));
      obstacle1.push_back(RVO::Vector2(15, 10));
      obstacle1.push_back(RVO::Vector2(15, 15));
      obstacle1.push_back(RVO::Vector2(10, 15));
      obstacles.push_back(obstacle1);

      // Astart
      // 搜索参数
      AStar::Params param;
      param.width = 10;
      param.height = 10;
      param.corner = false;
      param.start = AStar::Vec2(0, 0);
      param.end = AStar::Vec2(9, 9);
      param.can_pass = [&](const AStar::Vec2 &pos) -> bool
      {
          return maps[pos.y][pos.x] == 0;
      };

      // 执行搜索
      BlockAllocator allocator;
      AStar algorithm(&allocator);
      // 得到每一步的 goal
      std::vector<AStar::Vec2> path = algorithm.find(param);
      for(int i = 0; i < path.size(); ++i) {
        cout << path[i].x << " " << path[i].y << endl;
      }

      // TO DO
      // 优化一：
      // 进行路径的合并
      // param.start = AStar::Vec2(0, 0);
      // param.end = AStar::Vec2(9, 9);
      std::vector<RVO::Vector2> CompressedPath;
      for(int i = 0; i < path.size(); ++i) {
        CompressedPath.push_back(RVO::Vector2(path[i].x, path[i].y));
      }
      
      for(int i = 0; i < CompressedPath.size(); ++i) {
        cout << CompressedPath[i].x() << " " << CompressedPath[i].y() << endl;
      }


      // return CompressedPath;
      // 定义障碍物的位置
      // std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;
      // obstacle1.push_back(RVO::Vector2(5, 5));
      // obstacle1.push_back(RVO::Vector2(5, -5));
      // obstacle1.push_back(RVO::Vector2(-5, -5));
      // obstacle1.push_back(RVO::Vector2(-5, 5));
      // obstacles.push_back(obstacle1);
    }

    // 对场景中的所有 Agent 设置 偏好速度
    set_preferred_velocities();
    // return {};
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
    /* Specify the global time step of the simulation. */
    simulator->setTimeStep(dt);
    simulator->doStep();
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

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ui = ImGui::CreateContext();
    if (!ui) {
      std::printf("Failed to create ImGui context\n");
      terminate();
      return false;
    }

    // Setup Platform/Renderer bindings
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

  // 这个是啥？
  RVO::Vector2 toScreenSpace(const RVO::Vector2& point)
  {
    return {
      width / 2 + options.offset_x + point.x() * options.scale,
      height / 2 + options.offset_y - point.y() * options.scale,
    };
  }

  // 这个又是啥？
  RVO::Vector2 fromScreenSpace(const RVO::Vector2& point)
  {
    return RVO::Vector2{
      point.x() - width / 2 - options.offset_x,
      -point.y() + height / 2 + options.offset_y,
    } / options.scale;
  }

  // 画？
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

    // Obstacles
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

    // Staging obstacle
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
    simulation.initialize(simulation_options);
  }
  virtual ~App() = default;

  bool main_loop()
  {
    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<float>>(
      now - time_stamp);
    time_stamp = now;

    renderer.draw(dt.count(), simulation, simulation_options);

    if (simulation_options.run_simulation) {
      simulation.set_preferred_velocities();
      simulation.step(simulation_options.time_scale * dt.count());
    }
    

    SDL_Event event;
    // 这边是在SDL 在监控用户的行为
    while (SDL_PollEvent(&event)) {
      /* Add by Chen.Yu. */
      // 手动添加障碍物
      // simulation.staging_obstacle.emplace_back(renderer.fromScreenSpace(
      //   RVO::Vector2(-50, -50)));
      // simulation.staging_obstacle.emplace_back(renderer.fromScreenSpace(
      //   RVO::Vector2(-1, 1)));
      // simulation.staging_obstacle.emplace_back(renderer.fromScreenSpace(
      //   RVO::Vector2(1, 1)));
      // simulation.staging_obstacle.emplace_back(renderer.fromScreenSpace(
      //       RVO::Vector2(1, -1)));
      // simulation.commit_obstacle();
      ImGui_ImplSDL2_ProcessEvent(&event);
      uint32_t width, height;
      renderer.resolution(width, height);
      if (event.type == SDL_QUIT) {
        return false;
      } else if (event.type == SDL_MOUSEBUTTONDOWN &&
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
      } else if (event.type == SDL_MOUSEWHEEL &&
                 !renderer.ui_want_capture_mouse()) {
        renderer.options.scale += event.wheel.y * 0.01 * renderer.options.scale;
        if (renderer.options.scale < 0) {
          renderer.options.scale = 0.0f;
        }
      } else if (event.type == SDL_MOUSEMOTION &&
                 event.motion.state == SDL_PRESSED &&
                 !renderer.ui_want_capture_mouse()) {
        renderer.options.offset_x += event.motion.xrel;
        renderer.options.offset_y += event.motion.yrel;
      } else if (event.type == SDL_KEYDOWN &&
                 !renderer.ui_want_capture_keyboard()) {
        switch (event.key.keysym.scancode) {
          case SDL_SCANCODE_ESCAPE:
            return false;
          case SDL_SCANCODE_SPACE:
            simulation_options.run_simulation =
              !simulation_options.run_simulation;
            break;
          case SDL_SCANCODE_BACKSPACE:
            simulation.initialize(simulation_options);
            break;
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
    while (main_loop()) {
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
  Renderer renderer;
  Simulation::options_t simulation_options;
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

int
main(int argc, char* argv[])
{
  App app;
  return app.run();
}
