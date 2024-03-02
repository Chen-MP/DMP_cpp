#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;

int main() {
    std::wstring pythonHome = L"F:\\Python38";
    Py_SetPythonHome(pythonHome.c_str());
    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {2, 3, 4, 5, 6};

    double marker_size = 10.0;
    std::map<std::string, std::string> keywords = {{"color", "blue"}, {"label", "data"}};

    plt::scatter(x, y, marker_size, keywords);
    plt::show();

    return 0;
}