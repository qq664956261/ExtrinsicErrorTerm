#include"ExtrinsicErrorTerm/ExtrinsicErrorTerm.h"

int main()
{
    ExtrinsicErrorTerm::Ptr pExtrinsicErrorTerm;
    pExtrinsicErrorTerm = std::shared_ptr<ExtrinsicErrorTerm>(new ExtrinsicErrorTerm());
    pExtrinsicErrorTerm->readPose(".");
    pExtrinsicErrorTerm->readSonarWaveData(".");
    pExtrinsicErrorTerm->buildMap();
    pExtrinsicErrorTerm->align();
    return 0;
}