#include "ExtrinsicErrorTerm/Relocalization.h"

int main()
{
    Relocalization::Ptr pRelocalization;
    pRelocalization = std::shared_ptr<Relocalization>(new Relocalization());
    std::vector<std::vector<double>> Poses;
    std::deque<std::pair<int, double>> PoseTimeStamp;
    pRelocalization->readPose("./ArmyOdom.txt", Poses, PoseTimeStamp);
    pRelocalization->setPoses(Poses, PoseTimeStamp);
    std::vector<std::vector<double>> SonarWaveDatas;
    std::deque<std::pair<int, double>> SonarWaveTimeStamp;
    pRelocalization->readSonarWaveData("./ArmyUltra.txt", SonarWaveDatas, SonarWaveTimeStamp);
    pRelocalization->setSonarData(SonarWaveDatas, SonarWaveTimeStamp);
    pRelocalization->buildMultiFrame();
    pRelocalization->reloc();

    return 0;
}