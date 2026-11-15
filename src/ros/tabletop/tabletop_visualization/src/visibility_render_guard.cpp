#include <map>
#include <iostream>
#include <optional>
#include <string>

#include <QDateTime>
#include <QFile>
#include <QTimer>
#include <QWidget>

#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>

#include "tabletop_visualization/visibility_policy.hpp"

namespace tabletop_visualization
{
class VisibilityRenderGuard : public rviz_common::Display
{
public:
  ~VisibilityRenderGuard() override {restore();}

protected:
  void onInitialize() override
  {
    // This independent GUI timer keeps running even when OGRE targets are off.
    timer_ = new QTimer(this);
    timer_->setInterval(50);
    QObject::connect(timer_, &QTimer::timeout, this, [this]() {refresh();});
  }

  void onEnable() override {timer_->start();}
  void onDisable() override
  {
    timer_->stop();
    restore();
  }

private:
  Ogre::RenderSystem * renderSystem() const
  {
    auto * root = Ogre::Root::getSingletonPtr();
    return root ? root->getRenderSystem() : nullptr;
  }

  void restore()
  {
    if (auto * system = renderSystem()) {
      auto targets = system->getRenderTargetIterator();
      while (targets.hasMoreElements()) {
        auto * target = targets.getNext();
        auto found = paused_.find(target->getName());
        if (found != paused_.end()) {
          target->setActive(found->second);
        }
      }
    }
    paused_.clear();
  }

  void refresh()
  {
    auto * manager = dynamic_cast<rviz_common::VisualizationManager *>(context_);
    if (!manager || !manager->getRenderPanel()) {
      restore();
      return;
    }
    auto path = qEnvironmentVariable("TABLETOP_NOVNC_VISIBILITY_FILE");
    if (path.isEmpty()) {
      path = "/tabletop/.cache/tabletop/novnc-visibility.json";
    }
    QFile file(path);
    const auto document = file.open(QIODevice::ReadOnly) ?
      QJsonDocument::fromJson(file.readAll()) : QJsonDocument();
    auto * window = manager->getRenderPanel()->window();
    auto title = window->windowTitle();
    // Qt substitutes this marker in the native X11 title, but not windowTitle().
    title.replace("[*]", window->isWindowModified() ? "*" : "");
    const bool visible = shouldRender(
      document, title, QDateTime::currentMSecsSinceEpoch() / 1000.0);
    if (visible) {
      if (!paused_.empty()) {
        restore();
        context_->queueRender();
      }
    } else if (auto * system = renderSystem()) {
      auto targets = system->getRenderTargetIterator();
      while (targets.hasMoreElements()) {
        auto * target = targets.getNext();
        // Only actual render windows, never textures used for picking/shadows.
        if (dynamic_cast<Ogre::RenderWindow *>(target)) {
          paused_.emplace(target->getName(), target->isActive());
          target->setActive(false);
        }
      }
    }
    if (!last_visible_ || visible != *last_visible_) {
      std::cout << "[VIEW_RENDER] RViz: " << (visible ? "active" : "paused (hidden)")
                << std::endl;
      last_visible_ = visible;
    }
  }

  QTimer * timer_ = nullptr;
  std::map<std::string, bool> paused_;
  std::optional<bool> last_visible_;
};
}  // namespace tabletop_visualization

PLUGINLIB_EXPORT_CLASS(tabletop_visualization::VisibilityRenderGuard, rviz_common::Display)
