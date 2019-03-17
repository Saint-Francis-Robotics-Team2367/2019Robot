#include<SmartSender.h>

void SmartSender::addNumber(double * num, std::string name) 
{
    numbers.push_back(num);
    names.push_back(name);
}

void SmartSender::putNumbers()
{
    auto numIter = numbers.begin();
    auto nameIter = names.begin();

    while(numIter != numbers.end())
    {
        frc::SmartDashboard::PutNumber(*nameIter, **numIter);
        numIter++;
        nameIter++;
    }
}

void SmartSender::getNumbers()
{
    auto numIter = numbers.begin();
    auto nameIter = names.begin();

    while(numIter != numbers.end())
    {
        **numIter = frc::SmartDashboard::GetNumber(*nameIter, **numIter);
        numIter++;
        nameIter++;
    }
}