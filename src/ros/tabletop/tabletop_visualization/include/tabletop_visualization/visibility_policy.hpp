#pragma once

#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

namespace tabletop_visualization
{
// Match the Python dashboard policy: only an explicit, fresh false can pause.
inline bool shouldRender(const QJsonDocument & document, const QString & title, double now)
{
  const auto root = document.object();
  const auto updated = root.value("updated_epoch");
  const double age = now - updated.toDouble();
  if (root.value("version").toInt() != 1 || !updated.isDouble() || !(age >= 0 && age <= 3)) {
    return true;
  }
  const auto value = root.value("windows").toObject().value(title);
  return !value.isBool() || value.toBool();
}
}  // namespace tabletop_visualization
