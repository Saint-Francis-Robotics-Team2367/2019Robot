#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>

class SmartSender
{
    private:
    std::vector<std::string> names = std::vector<std::string>();
    std::vector<double*> numbers = std::vector<double*>();
    
    public:
    void addNumber(double * num, std::string name);
    void putNumbers();
    void getNumbers();
};