// Command posecloud-plan demonstrates the PoseCloud API against the offline
// motion planner (no robot/hardware required), using a 5-DoF arm.
//
// It loads a kinematic model from a .json or .urdf file, then plans to a goal
// pose twice: once requiring the EXACT pose, and once with a PoseCloud that
// gives leeway on the orientation a 5-DoF arm physically cannot reach.
// Comparing the two outcomes shows why leeway matters on a 5-DoF arm.
//
// By default it runs against the in-repo DOFBOT model so it works out of the
// box. DOFBOT shares the SO-ARM101's morphology: a base yaw, three parallel
// pitch joints, and a wrist roll. To run against the SO-ARM101 instead, grab
// its URDF and point --model at it (note its goal may need re-tuning, since the
// URDF is in meters and the link lengths differ):
//
//	curl -L -o so101.urdf \
//	  https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf
//	go run ./examples/posecloud/plan --model so101.urdf --name so101
//
// Default (DOFBOT, runs with no extra setup):
//
//	go run ./examples/posecloud/plan
//
// # WHY LEEWAY HELPS A 5-DoF ARM
//
// A full 6-DoF pose needs 3 positional + 3 orientational DoF. A 5-DoF arm of
// this "pan + parallel-pitch + roll" type spends its joints as: the base yaw
// plus three parallel pitches reach a 3D position and set the in-plane pitch of
// the tool, and the wrist roll spins the tool about its own axis. That leaves it
// ONE orientation DoF short: the base yaw fixes BOTH the position's azimuth and
// the plane the tool can point in, so the end effector's approach direction is
// locked to the vertical plane containing the base axis and the target. It
// cannot tilt the approach "out of plane" (with this arm at world origin and the
// target on +X, that means it can't produce an OY component in its approach).
//
// HOW TO EXPRESS THAT AS A PoseCloud. The leeways are evaluated on the *relative*
// orientation between the goal and a candidate, expressed as an orientation
// vector + theta. They are therefore NOT naive independent world axes:
//
//	OX, OY  bound how far (and in which direction) the candidate's approach
//	        axis may tilt away from the goal's approach axis
//	OZ      bounds |1 - OZ| of that relative vector, i.e. the overall approach-
//	        axis misalignment; it ranges [0, 2], so OZ:2 permits any tilt
//	Theta   bounds roll about the approach axis (degrees); Theta:180 frees roll
//
// The cleanest TRUE single-axis leeway is Theta. To free the *pointing*
// direction you must relax OX/OY (tilt direction) AND OZ (tilt magnitude)
// together -- tilting the approach necessarily reduces OZ. So the practical,
// robust recipe for a 5-DoF arm is "match the position, let the tool point
// however it can": a cloud with OX:1, OY:1, OZ:2, Theta:180.
package main

import (
	"context"
	"flag"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

func main() {
	modelPath := flag.String("model", "components/arm/sim/kinematics/dofbot.json",
		"path to a .json or .urdf kinematic model (e.g. an SO-ARM101 URDF)")
	modelName := flag.String("name", "arm", "frame name to give the loaded model")
	flag.Parse()

	logger := logging.NewLogger("posecloud-plan")
	ctx := context.Background()

	// KinematicModelFromFile handles both .json and .urdf models.
	model, err := referenceframe.KinematicModelFromFile(*modelPath, *modelName)
	if err != nil {
		logger.Fatalw("could not load model", "path", *modelPath, "error", err)
	}
	logger.Infow("loaded model", "name", model.Name(), "dof", len(model.DoF()))

	// Build a frame system with the arm attached to the world origin.
	fs := referenceframe.NewEmptyFrameSystem("planning")
	if err := fs.AddFrame(model, fs.World()); err != nil {
		logger.Fatalw("could not add model to frame system", "error", err)
	}

	// Start from a slightly bent configuration that already sits near the target
	// region. This keeps path planning fast for the demo so we can focus on the
	// IK feasibility difference that the PoseCloud makes. (Starting from the
	// fully extended zero pose works too, but is slower to connect.) The bent
	// values are applied to the first few joints; any extra joints (e.g. a
	// gripper joint in a URDF) stay at zero.
	startInputs := make([]referenceframe.Input, len(model.DoF()))
	for i, v := range []referenceframe.Input{0, -0.5, 0.8, 0.5} {
		if i < len(startInputs) {
			startInputs[i] = v
		}
	}
	startCfg := referenceframe.FrameSystemInputs{*modelName: startInputs}

	// The goal pose, expressed in the world frame. The position (millimeters) is
	// well within the DOFBOT's reach. The target sits along +X, so the arm's
	// radial plane is the X-Z plane; an in-plane approach has OY=0. We ask for an
	// approach with a sizeable sideways (OY) component, i.e. tilted OUT of that
	// plane -- exactly what a 5-DoF arm of this type cannot achieve.
	goalPose := spatialmath.NewPose(
		r3.Vector{X: 210, Y: 0, Z: 71},
		&spatialmath.OrientationVectorDegrees{OX: 0.6, OY: 0.5, OZ: 0.6},
	)

	// CASE A: demand the exact pose (no leeway). On a 5-DoF arm this fails for an
	// out-of-plane orientation: no joint configuration can reach it, so IK
	// produces zero solutions.
	logger.Info("CASE A: planning to the EXACT goal pose (no leeway)")
	planA, errA := plan(ctx, logger, fs, *modelName, startCfg, goalPose, nil)
	report(logger, planA, errA)

	// CASE B: same goal position, but let the tool point however the arm can by
	// freeing the whole approach orientation (see the package doc for why this is
	// OX:1, OY:1, OZ:2, Theta:180 rather than a single field). The planner now
	// finds an in-plane orientation that satisfies the relaxed goal.
	cloud := &referenceframe.PoseCloud{OX: 1, OY: 1, OZ: 2, Theta: 180}
	logger.Infow("CASE B: planning to the same goal WITH a PoseCloud", "cloud", cloud)
	planB, errB := plan(ctx, logger, fs, *modelName, startCfg, goalPose, cloud)
	report(logger, planB, errB)
}

// plan issues a single PlanMotion request to move modelName's end effector to
// goalPose (expressed in the world frame). If cloud is non-nil the goal is
// fuzzy: any pose within the cloud is accepted.
func plan(
	ctx context.Context,
	logger logging.Logger,
	fs *referenceframe.FrameSystem,
	modelName string,
	startCfg referenceframe.FrameSystemInputs,
	goalPose spatialmath.Pose,
	cloud *referenceframe.PoseCloud,
) (motionplan.Plan, error) {
	var goal *referenceframe.PoseInFrame
	if cloud == nil {
		goal = referenceframe.NewPoseInFrame(referenceframe.World, goalPose)
	} else {
		goal = referenceframe.NewPoseInFrameWithGoalCloud(referenceframe.World, goalPose, cloud)
	}

	p, _, err := armplanning.PlanMotion(ctx, logger, &armplanning.PlanRequest{
		FrameSystem: fs,
		Goals: []*armplanning.PlanState{
			armplanning.NewPlanState(referenceframe.FrameSystemPoses{modelName: goal}, nil),
		},
		StartState: armplanning.NewPlanState(nil, startCfg),
		// Keep the example snappy: cap IK/planning so an infeasible exact goal
		// returns quickly instead of grinding for minutes.
		PlannerOptions: &armplanning.PlannerOptions{Timeout: 10},
	})
	return p, err
}

func report(logger logging.Logger, p motionplan.Plan, err error) {
	if err != nil {
		logger.Infow("  -> NO PLAN (goal unreachable as specified)", "error", err)
		return
	}
	logger.Infow("  -> SUCCESS", "waypoints", len(p.Trajectory()))
}
