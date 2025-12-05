#include "util.h"
#include <cctype>

bool isNumber(const std::string& s) {
    bool hasDigit = false;
    bool hasDot = false;

    if (s.empty())
        return false;

    int i = 0;

    // Optional sign at index 0
    if (s[0] == '-' || s[0] == '+') 
        i = 1;

    for (; i < (int)s.size(); i++) {
        if (std::isdigit(s[i])) {
            hasDigit = true;
        }
        else if (s[i] == '.' && !hasDot) {
            hasDot = true;
        }
        else {
            return false;    // any other character = invalid
        }
    }

    return hasDigit;         // must contain at least one number
}
