// =============================================================================
// SliderText.cpp
// =============================================================================
#include "viewer/core/SliderText.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace viewer {

namespace {

/// Descida recursiva: expr = termo {(+|-) termo}; termo = fator {(*|/) fator}; fator = [+|-] fator | número | '(' expr ')'.
class ExprParser {
public:
    explicit ExprParser(const std::string& t) : s_(t) {}

    bool run(double& out) {
        double v = 0.0;
        if (!expr(v, 0)) return false;
        skipSpaces();
        // Sobra permitida: só sufixo de unidade (sem algarismos, operadores ou parênteses).
        for (; i_ < s_.size(); ++i_) {
            const unsigned char c = static_cast<unsigned char>(s_[i_]);
            if ((c >= '0' && c <= '9') || c == '+' || c == '-' || c == '*' || c == '/' || c == '(' || c == ')' || c == '.' || c == ',') return false;
        }
        if (!std::isfinite(v)) return false;
        out = v;
        return true;
    }

private:
    const std::string& s_;
    std::size_t i_ = 0;

    void skipSpaces() { while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t')) ++i_; }
    bool peek(char c) { skipSpaces(); return i_ < s_.size() && s_[i_] == c; }

    bool expr(double& out, int depth) {
        if (depth > 64) return false;
        double v = 0.0;
        if (!term(v, depth)) return false;
        for (;;) {
            if (peek('+')) { ++i_; double r = 0.0; if (!term(r, depth)) return false; v += r; }
            else if (peek('-')) { ++i_; double r = 0.0; if (!term(r, depth)) return false; v -= r; }
            else break;
            if (!std::isfinite(v)) return false;
        }
        out = v;
        return true;
    }

    bool term(double& out, int depth) {
        double v = 0.0;
        if (!factor(v, depth)) return false;
        for (;;) {
            if (peek('*')) { ++i_; double r = 0.0; if (!factor(r, depth)) return false; v *= r; }
            else if (peek('/')) { ++i_; double r = 0.0; if (!factor(r, depth) || r == 0.0) return false; v /= r; }
            else break;
            if (!std::isfinite(v)) return false;
        }
        out = v;
        return true;
    }

    bool factor(double& out, int depth) {
        if (depth > 64) return false;
        skipSpaces();
        if (i_ >= s_.size()) return false;
        if (s_[i_] == '+' || s_[i_] == '-') {
            const bool neg = s_[i_] == '-';
            ++i_;
            double v = 0.0;
            if (!factor(v, depth + 1)) return false;
            out = neg ? -v : v;
            return true;
        }
        if (s_[i_] == '(') {
            ++i_;
            double v = 0.0;
            if (!expr(v, depth + 1)) return false;
            if (!peek(')')) return false;
            ++i_;
            out = v;
            return true;
        }
        return number(out);
    }

    bool number(double& out) {
        const std::size_t start = i_;
        std::string t;
        bool digits = false, dot = false;
        while (i_ < s_.size()) {
            const char c = s_[i_];
            if (c >= '0' && c <= '9') { digits = true; t += c; ++i_; }
            else if ((c == '.' || c == ',') && !dot) { dot = true; t += '.'; ++i_; }
            else break;
        }
        if (!digits) { i_ = start; return false; }
        // expoente só se vier seguido de algarismos ("1e3", "2E-2"); senão o 'e' é sufixo de unidade.
        if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
            std::size_t k = i_ + 1;
            if (k < s_.size() && (s_[k] == '+' || s_[k] == '-')) ++k;
            if (k < s_.size() && s_[k] >= '0' && s_[k] <= '9') {
                t += 'e';
                for (std::size_t m = i_ + 1; m < k; ++m) t += s_[m];
                while (k < s_.size() && s_[k] >= '0' && s_[k] <= '9') t += s_[k++];
                i_ = k;
            }
        }
        char* end = nullptr;
        const double v = std::strtod(t.c_str(), &end);
        if (end == t.c_str() || !std::isfinite(v)) return false;
        out = v;
        return true;
    }
};

} // namespace

bool evalExpression(const std::string& text, double& out) {
    ExprParser p(text);
    return p.run(out);
}

bool parseNumber(const std::string& text, double& out) { return evalExpression(text, out); }

bool sliderValueFromText(const std::string& text, const SliderUnit& unit, float lo, float hi, float& out) {
    double typed = 0.0;
    if (!parseNumber(text, typed)) return false;
    const double scale = unit.scale != 0.0f && std::isfinite(unit.scale) ? static_cast<double>(unit.scale) : 1.0;
    double nv = typed / scale;
    if (unit.square) nv = nv < 0.0 ? 0.0 : std::sqrt(nv);
    nv = std::min(static_cast<double>(hi), std::max(static_cast<double>(lo), nv));
    out = static_cast<float>(nv);
    return true;
}

std::string sliderTextFromValue(float v, const SliderUnit& unit) {
    const double shown = (unit.square ? static_cast<double>(v) * static_cast<double>(v) : static_cast<double>(v)) * static_cast<double>(unit.scale);
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.5g", std::isfinite(shown) ? shown : 0.0);
    return buf;
}

std::vector<std::string> wrapText(const std::string& text, int maxWidth, int maxLines, const std::function<int(const std::string&)>& measure) {
    std::vector<std::string> lines;
    if (maxWidth <= 0 || maxLines <= 0 || text.empty() || !measure) return lines;
    std::string rest = text;
    while (!rest.empty() && static_cast<int>(lines.size()) < maxLines) {
        std::size_t n = rest.size();
        while (n > 1 && measure(rest.substr(0, n)) > maxWidth) --n;
        if (n < rest.size()) {   // não coube inteiro: ponto de quebra natural no último terço
            for (std::size_t k = n; k > n * 2 / 3 && k > 1; --k) {
                const char c = rest[k - 1];
                if (c == '/' || c == '\\' || c == '_' || c == ' ') { n = k; break; }
            }
        }
        std::string line = rest.substr(0, n);
        rest.erase(0, n);
        if (static_cast<int>(lines.size()) == maxLines - 1 && !rest.empty()) {   // última linha permitida e ainda sobra texto
            while (line.size() > 1 && measure(line + "..") > maxWidth) line.pop_back();
            line += "..";
        }
        lines.push_back(line);
    }
    return lines;
}

} // namespace viewer
