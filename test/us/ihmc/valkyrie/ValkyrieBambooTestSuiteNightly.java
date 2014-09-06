package us.ihmc.valkyrie;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
   ValkyrieObstacleCourseRampsTest.class,
   ValkyrieObstacleCourseFlatTest.class,
   ValkyrieObstacleCourseTrialsTerrainTest.class,
   ValkyrieObstacleCourseEveryBuildTest.class,
   ValkyriePushRecoveryStandingTest.class,
   ValkyriePushRecoveryTest.class,
   ValkyriePushRecoveryMultiStep.class
})

public class ValkyrieBambooTestSuiteNightly
{
   
}

