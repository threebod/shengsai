#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/axis6_test.h"
int main(void)
{
    MechanismPose origin = {-900, 530, 2773, 1000, 200, 2000, 240, 1800};
    MechanismAction actions[2];
    unsigned i;
    assert(axis6TestBuild(&origin, actions));
    assert(actions[0].pose.horizontalDmm == -850 && actions[1].pose.horizontalDmm == -900);
    for (i = 0; i < 2; ++i) {
        assert(actions[i].type == MECHANISM_ACTION_POSE && actions[i].waitMs == 0);
        assert(actions[i].pose.horizontalRpm == 30 && actions[i].pose.horizontalAccel == 50);
        assert(actions[i].pose.liftDmm == origin.liftDmm && actions[i].pose.turretDdeg == origin.turretDdeg);
        assert(mechanismPoseValid(&actions[i].pose));
    }
    assert(origin.horizontalDmm == -900 && origin.horizontalRpm == 1000);
    origin.horizontalDmm = 800;assert(axis6TestBuild(&origin, actions));
    assert(actions[0].pose.horizontalDmm == 850 && actions[1].pose.horizontalDmm == 800);
    origin.horizontalDmm = 801;assert(!axis6TestBuild(&origin, actions));
    origin.horizontalDmm = -1220;assert(axis6TestBuild(&origin, actions));
    origin.horizontalDmm = -1221;assert(!axis6TestBuild(&origin, actions));
    puts("PASS axis 6 test: 5mm outward/return, low speed, bounds, unchanged lift/turret and original pose");
    return 0;
}
