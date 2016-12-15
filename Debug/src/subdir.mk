################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Agent.cpp \
../src/BuildingTemplate.cpp \
../src/Contour.cpp \
../src/Costmap.cpp \
../src/CostmapCoordination.cpp \
../src/CostmapPlanning.cpp \
../src/CumulativePathPlanner.cpp \
../src/Frontier.cpp \
../src/Graph.cpp \
../src/GraphCoordination.cpp \
../src/GraphNode.cpp \
../src/GraphPlanning.cpp \
../src/HidingAgent.cpp \
../src/Inference.cpp \
../src/Market.cpp \
../src/Observer.cpp \
../src/PRMGraph.cpp \
../src/Pose.cpp \
../src/RoomTemplate.cpp \
../src/SearchPlan.cpp \
../src/ThinGraph.cpp \
../src/TreeNode.cpp \
../src/World.cpp \
../src/main_coordination.cpp 

OBJS += \
./src/Agent.o \
./src/BuildingTemplate.o \
./src/Contour.o \
./src/Costmap.o \
./src/CostmapCoordination.o \
./src/CostmapPlanning.o \
./src/CumulativePathPlanner.o \
./src/Frontier.o \
./src/Graph.o \
./src/GraphCoordination.o \
./src/GraphNode.o \
./src/GraphPlanning.o \
./src/HidingAgent.o \
./src/Inference.o \
./src/Market.o \
./src/Observer.o \
./src/PRMGraph.o \
./src/Pose.o \
./src/RoomTemplate.o \
./src/SearchPlan.o \
./src/ThinGraph.o \
./src/TreeNode.o \
./src/World.o \
./src/main_coordination.o 

CPP_DEPS += \
./src/Agent.d \
./src/BuildingTemplate.d \
./src/Contour.d \
./src/Costmap.d \
./src/CostmapCoordination.d \
./src/CostmapPlanning.d \
./src/CumulativePathPlanner.d \
./src/Frontier.d \
./src/Graph.d \
./src/GraphCoordination.d \
./src/GraphNode.d \
./src/GraphPlanning.d \
./src/HidingAgent.d \
./src/Inference.d \
./src/Market.d \
./src/Observer.d \
./src/PRMGraph.d \
./src/Pose.d \
./src/RoomTemplate.d \
./src/SearchPlan.d \
./src/ThinGraph.d \
./src/TreeNode.d \
./src/World.d \
./src/main_coordination.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


