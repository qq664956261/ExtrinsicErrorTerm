#include "ExtrinsicErrorTerm/Mapping.h"

int main()
{
    Mapping::Ptr pMapping;
    pMapping = std::shared_ptr<Mapping>(new Mapping());
    pMapping->readPose(".");
    pMapping->readSonarWaveData(".");

    return 0;
}