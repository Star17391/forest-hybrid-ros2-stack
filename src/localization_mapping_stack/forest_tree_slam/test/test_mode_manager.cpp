#include <gtest/gtest.h>

#include "forest_tree_slam/mode_manager.hpp"

using forest_tree_slam::ModeManager;
using forest_tree_slam::ModeManagerInputs;
using forest_tree_slam::SlamMode;

TEST(ModeManager, StartsInGroundOwningTf)
{
  ModeManager mm;
  EXPECT_EQ(mm.mode(), SlamMode::GROUND);
  EXPECT_TRUE(mm.owns_map_to_odom());
}

TEST(ModeManager, TakeoffTriggersSnapshotAndDropsAuthority)
{
  ModeManager mm;
  ModeManagerInputs in;
  in.locomotion_aerial = true;
  const auto events = mm.update(in);
  EXPECT_TRUE(events.request_takeoff_snapshot);
  EXPECT_EQ(mm.mode(), SlamMode::AERIAL);
  EXPECT_FALSE(mm.owns_map_to_odom());
}

TEST(ModeManager, LandingTriggersMandatoryRelocalizationWhenGnssBad)
{
  ModeManager mm;
  ModeManagerInputs in;
  in.locomotion_aerial = true;
  mm.update(in);  // takeoff

  in.hop_in_progress = true;
  mm.update(in);
  EXPECT_EQ(mm.mode(), SlamMode::AERIAL);

  in.hop_in_progress = false;
  in.hop_done = true;
  in.gnss_good = false;
  const auto events = mm.update(in);
  EXPECT_TRUE(events.request_relocalization);
  EXPECT_TRUE(events.relocalization_mandatory);
  EXPECT_EQ(mm.mode(), SlamMode::RELOCALIZING);
  EXPECT_FALSE(mm.owns_map_to_odom());
}

TEST(ModeManager, OptionalRelocalizationFallsBackToGroundOnRejection)
{
  ModeManager mm;
  ModeManagerInputs in;
  in.locomotion_aerial = true;
  mm.update(in);
  in.hop_done = true;
  in.gnss_good = true;  // opcional
  const auto events = mm.update(in);
  EXPECT_FALSE(events.relocalization_mandatory);

  mm.notify_relocalization_result(false);  // rejeitado, mas era opcional
  EXPECT_EQ(mm.mode(), SlamMode::GROUND);
  EXPECT_TRUE(mm.owns_map_to_odom());
}

TEST(ModeManager, MandatoryRelocalizationEscalatesToLostAfterRetries)
{
  forest_tree_slam::ModeManagerParams params;
  params.max_mandatory_relocalization_attempts = 2;
  ModeManager mm(params);
  ModeManagerInputs in;
  in.locomotion_aerial = true;
  mm.update(in);
  in.hop_done = true;
  in.gnss_good = false;
  mm.update(in);

  mm.notify_relocalization_result(false);
  EXPECT_EQ(mm.mode(), SlamMode::RELOCALIZING);
  mm.notify_relocalization_result(false);
  EXPECT_EQ(mm.mode(), SlamMode::LOST);
  EXPECT_FALSE(mm.owns_map_to_odom());
}

TEST(ModeManager, SuccessfulRelocalizationReturnsToGround)
{
  ModeManager mm;
  ModeManagerInputs in;
  in.locomotion_aerial = true;
  mm.update(in);
  in.hop_done = true;
  in.gnss_good = false;
  mm.update(in);

  mm.notify_relocalization_result(true);
  EXPECT_EQ(mm.mode(), SlamMode::GROUND);
  EXPECT_TRUE(mm.owns_map_to_odom());
  EXPECT_FALSE(mm.pose_frozen());
}

TEST(ModeManager, DegradesThenGoesLostOnProlongedAssociationLoss)
{
  forest_tree_slam::ModeManagerParams params;
  params.degraded_after_scans = 3;
  params.lost_after_scans = 6;
  ModeManager mm(params);
  ModeManagerInputs in;

  in.scans_since_any_association = 4;
  mm.update(in);
  EXPECT_EQ(mm.mode(), SlamMode::GROUND);
  EXPECT_TRUE(mm.pose_frozen());  // DEGRADED: continua GROUND mas congela TF

  in.scans_since_any_association = 7;
  mm.update(in);
  EXPECT_EQ(mm.mode(), SlamMode::LOST);
  EXPECT_FALSE(mm.owns_map_to_odom());
}

TEST(ModeManager, RecoversFromLostWhenAssociationResumesOnGround)
{
  forest_tree_slam::ModeManagerParams params;
  params.lost_after_scans = 2;
  ModeManager mm(params);
  ModeManagerInputs in;
  in.scans_since_any_association = 3;
  mm.update(in);
  ASSERT_EQ(mm.mode(), SlamMode::LOST);

  in.scans_since_any_association = 0;
  mm.update(in);
  EXPECT_EQ(mm.mode(), SlamMode::GROUND);
}
