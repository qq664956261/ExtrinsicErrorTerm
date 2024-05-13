#include "ExtrinsicErrorTerm/Relocalization.h"

int main()
{
    Relocalization::Ptr pRelocalization;
    pRelocalization = std::shared_ptr<Relocalization>(new Relocalization());
    pRelocalization->readPose(".");
    pRelocalization->readSonarWaveData(".");
    pRelocalization->buildMultiFrame();
    pRelocalization->reloc();

    return 0;
}