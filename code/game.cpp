/*
 * 地牢逃脱 (Dungeon Escape) - 控制台冒险游戏
 * 面向对象课程项目
 * 编译器要求：C++14 或以上
 * 依赖：ChGL.hpp（教师提供，需置于同一目录）
 */

#include "ChGL.hpp"
#include <vector>
#include <string>
#include <memory>
#include <queue>
#include <map>
#include <chrono>
#include <thread>
#include <cmath>

// ==================== 强类型枚举表示地形 ====================
enum class Tile : char {
    Wall   = '#',
    Floor  = '.',
    Mud    = '~',
    Key    = '$',
    Exit   = 'X',
    Player = 'P'
};

// ==================== 魔法数字集中管理 ====================
namespace Config {
    constexpr int VIEW_DISTANCE         = 7;    // 玩家视野半径（战争迷雾）
    constexpr int STALKER_NORMAL_DELAY  = 15;   // 追踪者平地冷却帧数（约0.5秒）
    constexpr int STALKER_MUD_DELAY     = 40;   // 追踪者泥泞冷却帧数（约1.3秒）
    constexpr int AMBUSHER_NORMAL_DELAY = 10;   // 伏击者平地冷却帧数
    constexpr int AMBUSHER_MUD_DELAY    = 30;   // 伏击者泥泞冷却帧数
    constexpr int AMBUSHER_ACTIVE_RANGE = 6;    // 伏击者激活距离（格）
    constexpr int FRAME_DURATION_MS     = 30;   // 游戏主循环帧间隔（毫秒）
}

// ==================== 轻量坐标结构 ====================
struct Point {
    int x, y;
    bool operator<(const Point& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

// ==================== 1. 地图类（封装地图数据与状态） ====================
class GameMap {
private:
    std::vector<std::string> layout;     // 地图布局（现代 string 代替 char[]）
    bool keyCollected = false;           // 私有状态：钥匙是否已拾取

public:
    GameMap() {
        layout = {
            "##############################",
            "#P..#.......#..........#....$#",
            "#.#.#.#####.#.########.#.###.#",
            "#.#...#...#.#.#......#...#...#",
            "#.#.###.#.#.#.#.####.#####.#.#",
            "#.#.....#.#...#.#~~#.......#.#",
            "#.#######.#####.####.#######.#",
            "#.......#...#......#.........#",
            "#.........#...~~~~.#........X#",
            "##############################"
        };
    }

    // 绘制地图（带战争迷雾）
    void drawTo(ChGL& gl, int px, int py) const {
        for (int y = 0; y < static_cast<int>(layout.size()); ++y) {
            for (int x = 0; x < static_cast<int>(layout[y].size()); ++x) {
                double dist = std::sqrt(std::pow(x - px, 2) + std::pow(y - py, 2));
                if (dist < Config::VIEW_DISTANCE) {
                    Tile tile = static_cast<Tile>(layout[y][x]);
                    // 若钥匙已拾取，原钥匙位置显示为地板
                    if (tile == Tile::Key && keyCollected) tile = Tile::Floor;
                    gl.setPixel(x, y, static_cast<char>(tile));
                } else {
                    gl.setPixel(x, y, ' ');   // 视野以外的迷雾
                }
            }
        }
    }

    // 检查坐标是否可通行（封装内部状态判断）
    bool isWalkable(int x, int y) const {
        if (y < 0 || y >= static_cast<int>(layout.size()) ||
            x < 0 || x >= static_cast<int>(layout[0].size()))
            return false;
        Tile tile = static_cast<Tile>(layout[y][x]);
        // 出口在获得钥匙前不可通行 → 双阶段任务目标
        if (tile == Tile::Exit && !keyCollected) return false;
        return tile != Tile::Wall;
    }

    // 获取地形代价（泥泞为10，其余为1，用于 Dijkstra 加权）
    int getCost(int x, int y) const {
        return (static_cast<Tile>(layout[y][x]) == Tile::Mud) ? 10 : 1;
    }

    void collectKey() { keyCollected = true; }
    bool hasKey() const { return keyCollected; }
    bool isKey(int x, int y) const {
        return static_cast<Tile>(layout[y][x]) == Tile::Key;
    }
    bool isVictory(int x, int y) const {
        return static_cast<Tile>(layout[y][x]) == Tile::Exit && keyCollected;
    }
};

// ==================== 2. 路径寻找类（Dijkstra 加权寻路） ====================
class PathFinder {
public:
    struct Node {
        Point pos;
        int dist;
        bool operator>(const Node& other) const { return dist > other.dist; }
    };

    // 返回从起点到终点加权最短路径的下一步坐标
    static Point getNextStep(const GameMap& map, Point start, Point target) {
        if (start == target) return start;

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
        std::map<Point, int> min_dist;
        std::map<Point, Point> parent;

        pq.push({start, 0});
        min_dist[start] = 0;
        const int dx[] = {0, 0, 1, -1};
        const int dy[] = {1, -1, 0, 0};

        while (!pq.empty()) {
            Node current = pq.top();
            pq.pop();
            if (current.pos == target) break;

            for (int i = 0; i < 4; ++i) {
                Point next{current.pos.x + dx[i], current.pos.y + dy[i]};
                if (map.isWalkable(next.x, next.y) || next == target) {
                    // 核心：移动代价加权（泥泞代价更高，使 AI 可能绕行）
                    int new_dist = current.dist + map.getCost(next.x, next.y);
                    if (min_dist.find(next) == min_dist.end() || new_dist < min_dist[next]) {
                        min_dist[next] = new_dist;
                        parent[next] = current.pos;
                        pq.push({next, new_dist});
                    }
                }
            }
        }

        if (parent.find(target) == parent.end()) return start; // 无路可走则原地不动
        Point curr = target;
        while (!(parent[curr] == start)) curr = parent[curr];
        return curr;
    }
};

// ==================== 3. 实体抽象基类（多态根基） ====================
class Entity {
protected:
    int x, y;       // 坐标
    char icon;      // 显示符号（P / S / A）
public:
    Entity(int x, int y, char icon) : x(x), y(y), icon(icon) {}
    virtual ~Entity() = default;

    // 纯虚函数：派生类必须实现自己的更新逻辑（多态核心）
    virtual void update(GameMap& map, Point pPos) = 0;

    // 公用绘制：只有在玩家视野内才画出来
    void draw(ChGL& gl, int px, int py) {
        double dist = std::sqrt(std::pow(x - px, 2) + std::pow(y - py, 2));
        if (dist < Config::VIEW_DISTANCE)
            gl.setPixel(x, y, icon);
    }

    Point getPos() const { return {x, y}; }
};

// ==================== 4. 派生类：玩家 ====================
class Player : public Entity {
public:
    Player(int x, int y) : Entity(x, y, static_cast<char>(Tile::Player)) {}

    void update(GameMap& map, Point /*pPos*/) override {
        InputHandler ih;   // 来自 ChGL.hpp 的键盘输入封装
        if (ih.kbhit()) {
            int key = static_cast<unsigned char>(ih.getch());
            int nx = x, ny = y;
            bool moved = false;

            // 方向键为两字节扩展码，首字节可能为 224 或 0
            if (key == 224 || key == 0) {
                key = static_cast<unsigned char>(ih.getch());
                if (key == 72)      { ny--; moved = true; }   // 上
                else if (key == 80) { ny++; moved = true; }   // 下
                else if (key == 75) { nx--; moved = true; }   // 左
                else if (key == 77) { nx++; moved = true; }   // 右
            }

            // 通过地图接口检查通行，不直接访问内部数据（体现封装）
            if (moved && (map.isWalkable(nx, ny) || map.isVictory(nx, ny))) {
                x = nx; y = ny;
                if (map.isKey(x, y)) map.collectKey();   // 拾取钥匙，改变地图状态
            }
        }
    }
};

// ==================== 5. 派生类：追踪者（全程 Dijkstra 追击） ====================
class Stalker : public Entity {
private:
    int cd = 0;    // 冷却计数器，控制移动频率（用于游戏平衡）
public:
    Stalker(int x, int y) : Entity(x, y, 'S') {}

    void update(GameMap& map, Point pPos) override {
        // 根据地形选择不同延迟：泥泞中更慢
        int delay = (map.getCost(x, y) > 1) ? Config::STALKER_MUD_DELAY
                                            : Config::STALKER_NORMAL_DELAY;
        if (++cd >= delay) {
            cd = 0;
            Point next = PathFinder::getNextStep(map, {x, y}, pPos);
            x = next.x; y = next.y;
        }
    }
};

// ==================== 6. 派生类：伏击者（条件激活追击） ====================
class Ambusher : public Entity {
private:
    int cd = 0;    // 冷却计数器
public:
    Ambusher(int x, int y) : Entity(x, y, 'A') {}

    void update(GameMap& map, Point pPos) override {
        double dist = std::sqrt(std::pow(x - pPos.x, 2) + std::pow(y - pPos.y, 2));
        // 只有玩家进入激活范围才开始追击
        if (dist < Config::AMBUSHER_ACTIVE_RANGE) {
            int delay = (map.getCost(x, y) > 1) ? Config::AMBUSHER_MUD_DELAY
                                                : Config::AMBUSHER_NORMAL_DELAY;
            if (++cd >= delay) {
                cd = 0;
                Point next = PathFinder::getNextStep(map, {x, y}, pPos);
                x = next.x; y = next.y;
            }
        }
    }
};

// ==================== 7. 游戏引擎（主循环） ====================
class GameEngine {
private:
    ChGL gl;
    GameMap map;
    std::vector<std::unique_ptr<Entity>> actors;  // 多态实体容器（智能指针）
    bool running = true;
    Player* pRef = nullptr;                       // 裸指针仅用于快速访问玩家

public:
    GameEngine() : gl(80, 25, ' ') {
        auto p = std::make_unique<Player>(1, 1);
        pRef = p.get();
        actors.push_back(std::move(p));
        actors.push_back(std::make_unique<Stalker>(14, 5));
        actors.push_back(std::make_unique<Ambusher>(22, 7));
    }

    void run() {
        std::string msg;
        while (running) {
            Point pPos = pRef->getPos();

            // 多态遍历：统一调用 update，自动分发到各派生类
            for (auto& a : actors) {
                a->update(map, pPos);
                // 碰撞检测：敌人坐标与玩家重合即失败
                if (a.get() != pRef && a->getPos() == pPos) {
                    msg = "CAUGHT BY ENEMY!";
                    running = false;
                }
            }

            // 胜利检测：持钥且位于出口
            if (map.isVictory(pPos.x, pPos.y)) {
                msg = "YOU ESCAPED!";
                running = false;
            }

            // 渲染：清屏 → 画地图（迷雾）→ 画实体
            gl.clear();
            map.drawTo(gl, pPos.x, pPos.y);
            for (auto& a : actors) a->draw(gl, pPos.x, pPos.y);

            // HUD 状态栏
            std::string hint = map.hasKey() ? "Status: [KEY OK] GO TO EXIT"
                                            : "Status: [NO KEY] FIND KEY $";
            std::string tip  = "Use ARROW KEYS. Avoid Mud (~).";
            for (size_t i = 0; i < hint.size(); ++i) gl.setPixel((int)i, 11, hint[i]);
            for (size_t i = 0; i < tip.size();  ++i) gl.setPixel((int)i, 12, tip[i]);

            gl.show();  // 双缓冲输出，光标归位 → 无闪烁
            std::this_thread::sleep_for(std::chrono::milliseconds(Config::FRAME_DURATION_MS));
        }

        // 游戏结束画面
        gl.clear();
        for (size_t i = 0; i < msg.size(); ++i) gl.setPixel((int)i + 5, 5, msg[i]);
        gl.show();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
};

// ==================== 主函数 ====================
int main() {
    GameEngine game;
    game.run();
    return 0;
}
