// Package main demonstrates how to request arm/gripper motion toward a "fuzzy"
// goal pose using RDK's PoseCloud API.
//
// A PoseCloud expresses per-dimension *leeway* around a goal pose. Instead of
// demanding that the end effector reach one exact pose, you tell the motion
// planner "any pose within this cloud is equally acceptable". This relaxes
// inverse kinematics, which can make an otherwise-infeasible goal solvable --
// for example when the exact goal would put the arm in collision, or when the
// last few degrees of wrist roll simply don't matter for the task.
//
// The cloud has seven independent leeways, each a symmetric band [-Value, +Value]:
//
//	X, Y, Z   translational leeway in millimeters
//	OX, OY, OZ orientation-vector leeway (unitless, on the normalized unit sphere)
//	Theta      rotation about the orientation axis, in degrees
//
// IMPORTANT: the leeways are evaluated in the reference frame of the goal's
// PoseInFrame -- i.e. relative to the orientation of the object you are moving
// toward, not the world axes. Expressing the goal in the target object's frame
// is therefore the usual pattern.
//
// Usage (assumes a running viam-server with an arm and the builtin motion service):
//
//	go run ./examples/posecloud --address localhost:8080 --component my_arm
package main

import (
	"context"
	"flag"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

func main() {
	address := flag.String("address", "localhost:8080", "address of the viam-server to connect to")
	component := flag.String("component", "my_arm", "name of the arm/gripper component to move")
	flag.Parse()

	logger := logging.NewLogger("posecloud-example")
	ctx := context.Background()

	machine, err := client.New(ctx, *address, logger)
	if err != nil {
		logger.Fatal(err)
	}
	//nolint:errcheck
	defer machine.Close(ctx)

	// The builtin motion service plans and executes motion across the robot's frame system.
	motionService, err := motion.FromProvider(machine, "builtin")
	if err != nil {
		logger.Fatal(err)
	}

	// The exact pose we'd *like* the end effector to reach, in millimeters. Here
	// it is expressed in the world frame for simplicity; in a real task you'd
	// often express it in the frame of the object you are interacting with so
	// that the cloud leeways (below) are applied relative to that object.
	goalPose := spatialmath.NewPose(
		r3.Vector{X: 400, Y: 0, Z: 200},
		&spatialmath.OrientationVectorDegrees{OZ: -1}, // tool pointing straight down
	)

	// -----------------------------------------------------------------------
	// CASE 1 -- a single axis at FULL leeway.
	//
	// The cleanest "one axis, fully free" request is to let the tool spin
	// freely about its own orientation (approach) axis. That axis is Theta, in
	// degrees over [-Theta, +Theta]; Theta = 180 accepts ANY rotation about the
	// approach axis. This is the classic symmetric-gripper / welding-tool case:
	// we care where the tool points, but not how it is rolled about that point.
	//
	// Every other field is left at its zero value, i.e. NO leeway -- the planner
	// must still match X, Y, Z and the OX/OY/OZ pointing direction exactly.
	fullThetaLeeway := &referenceframe.PoseCloud{
		Theta: 180, // full rotational freedom about the orientation axis
	}

	destination := referenceframe.NewPoseInFrameWithGoalCloud(
		referenceframe.World, // frame the goal pose AND the leeways are expressed in
		goalPose,
		fullThetaLeeway,
	)

	logger.Info("CASE 1: requesting motion with full leeway on the tool's rotation (Theta) axis only")
	moved, err := motionService.Move(ctx, motion.MoveReq{
		ComponentName: *component,
		Destination:   destination,
	})
	if err != nil {
		logger.Fatalw("move failed", "error", err)
	}
	logger.Infow("move complete", "moved", moved)

	// -----------------------------------------------------------------------
	// Other ways to express leeway (shown for reference, not executed):
	//
	// Note there is no "infinity" sentinel. "Full" leeway means:
	//   - orientation component: OX/OY/OZ = 1.0 accepts any value of that
	//     component (orientation vectors are normalized to the unit sphere, so a
	//     magnitude of 1 covers the whole range).
	//   - rotation about the axis: Theta = 180 (degrees), as in CASE 1.
	//   - translation: there is no infinite value, so use a large magnitude in
	//     millimeters to approximate "anywhere along this axis".
	//
	// A few illustrative clouds:
	freeApproachDirection := &referenceframe.PoseCloud{OX: 1.0}
	freeAlongX := &referenceframe.PoseCloud{X: 1000}
	pickUpCupLeeway := &referenceframe.PoseCloud{
		// a little translational slack and free wrist roll, mirroring the
		// motionplan PoseCloud integration test
		X: 10, Y: 10, Z: 40, OX: 1.0, OY: 1.0, Theta: 45,
	}
	logger.Infow("reference clouds (not executed)",
		"freeApproachDirection", freeApproachDirection,
		"freeAlongX", freeAlongX,
		"pickUpCupLeeway", pickUpCupLeeway,
	)

	// The GoalCloud field is exported, so you can also set it on an existing
	// PoseInFrame instead of using the constructor:
	pif := referenceframe.NewPoseInFrame(referenceframe.World, goalPose)
	pif.GoalCloud = fullThetaLeeway
	_ = pif
}
