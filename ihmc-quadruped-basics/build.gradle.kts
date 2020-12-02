plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc.thirdparty.jinput:jinput:200128")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:euclid-frame:0.15.2")
   api("us.ihmc:euclid-frame-shape:0.15.2")
   api("us.ihmc:euclid-shape:0.15.2")
   api("us.ihmc:ihmc-yovariables:0.9.7")
   api("us.ihmc:ihmc-robot-description:0.20.1")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")

}

testDependencies {
   api("com.google.caliper:caliper:1.0-beta-2")

   api("us.ihmc:simulation-construction-set-tools-test:source")
}
