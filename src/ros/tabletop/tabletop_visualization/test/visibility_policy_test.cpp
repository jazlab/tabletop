#include <gtest/gtest.h>
#include "tabletop_visualization/visibility_policy.hpp"

using tabletop_visualization::shouldRender;

TEST(VisibilityPolicy, OnlyFreshExplicitFalsePauses)
{
  const auto doc = QJsonDocument::fromJson(
    R"({"version":1,"updated_epoch":100,"windows":{"RViz":false,"Other":true}})");
  EXPECT_FALSE(shouldRender(doc, "RViz", 101));
  EXPECT_TRUE(shouldRender(doc, "Other", 101));
  EXPECT_TRUE(shouldRender(doc, "Unknown", 101));
  EXPECT_TRUE(shouldRender(doc, "RViz", 104));
  EXPECT_TRUE(shouldRender(doc, "RViz", 99));
  EXPECT_TRUE(shouldRender(QJsonDocument(), "RViz", 101));
}

TEST(VisibilityPolicy, InvalidValuesFailOpen)
{
  for (const auto data : {
      R"({"version":2,"updated_epoch":100,"windows":{"RViz":false}})",
      R"({"version":1,"updated_epoch":true,"windows":{"RViz":false}})",
      R"({"version":1,"updated_epoch":100,"windows":{"RViz":0}})",
      R"({"version":1,"updated_epoch":100,"windows":{"RViz":"false"}})"})
  {
    EXPECT_TRUE(shouldRender(QJsonDocument::fromJson(data), "RViz", 101));
  }
}
