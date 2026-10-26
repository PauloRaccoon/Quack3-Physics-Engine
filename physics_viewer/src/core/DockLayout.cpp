// =============================================================================
// DockLayout.cpp
// =============================================================================
#include "viewer/core/DockLayout.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace viewer {

namespace {
bool okNumber(float v) { return std::isfinite(v); }
float clampf(float v, float lo, float hi) { return std::min(std::max(v, lo), std::max(lo, hi)); }
} // namespace

// ----------------------------------------------------------------------------------------------------
// DockLayout
// ----------------------------------------------------------------------------------------------------
void DockLayout::setWindow(float w, float h) {
    if (!okNumber(w) || !okNumber(h) || w < 1.0f || h < 1.0f) return;
    winW_ = w;
    winH_ = h;
    reclamp();
}

float DockLayout::maxWidthFor(DockSide s) const {
    const float otherW = s == DockSide::Left ? (rightCollapsed_ ? kCollapsed : rightW_) : (leftCollapsed_ ? kCollapsed : leftW_);
    const float byFraction = kMaxDockFraction * winW_;
    const float byViewport = winW_ - otherW - 2.0f * kSplitter - kMinViewport;
    return std::min(byFraction, byViewport);
}

void DockLayout::reclamp() {
    // Cada coluna limitada pela outra; duas passadas bastam (a segunda corrige o efeito da primeira).
    for (int pass = 0; pass < 2; ++pass) {
        leftW_ = clampf(leftW_, kMinDock, maxWidthFor(DockSide::Left));
        rightW_ = clampf(rightW_, kMinDock, maxWidthFor(DockSide::Right));
    }
}

void DockLayout::setCollapsed(DockSide s, bool c) {
    (s == DockSide::Left ? leftCollapsed_ : rightCollapsed_) = c;
    reclamp();
}

void DockLayout::setWidth(DockSide s, float w) {
    if (!okNumber(w)) return;
    if (s == DockSide::Left) leftW_ = clampf(w, kMinDock, maxWidthFor(DockSide::Left));
    else rightW_ = clampf(w, kMinDock, maxWidthFor(DockSide::Right));
}

float DockLayout::effectiveWidth(DockSide s) const {
    if (collapsed(s)) return kCollapsed;
    const float w = s == DockSide::Left ? leftW_ : rightW_;
    // Janela pequena demais: a coluna cede espaço para o viewport mínimo (pode chegar a 0).
    const float otherW = s == DockSide::Left ? (rightCollapsed_ ? kCollapsed : rightW_) : (leftCollapsed_ ? kCollapsed : leftW_);
    const float room = winW_ - otherW - 2.0f * kSplitter - kMinViewport;
    return std::max(0.0f, std::min(w, room));
}

RectF DockLayout::topBar() const { return RectF{0, 0, winW_, kTopBar}; }

RectF DockLayout::dock(DockSide s) const {
    const float w = effectiveWidth(s);
    const float h = std::max(0.0f, winH_ - kTopBar);
    return s == DockSide::Left ? RectF{0, kTopBar, w, h} : RectF{winW_ - w, kTopBar, w, h};
}

RectF DockLayout::splitter(DockSide s) const {
    if (collapsed(s) || effectiveWidth(s) <= 0.0f) return RectF{};
    const RectF d = dock(s);
    const float h = d.h;
    return s == DockSide::Left ? RectF{d.right(), kTopBar, kSplitter, h} : RectF{d.x - kSplitter, kTopBar, kSplitter, h};
}

RectF DockLayout::viewport() const {
    const RectF l = dock(DockSide::Left), r = dock(DockSide::Right);
    const float sl = splitter(DockSide::Left).w, sr = splitter(DockSide::Right).w;
    const float x = l.right() + sl;
    const float w = std::max(1.0f, r.x - sr - x);
    return RectF{x, kTopBar, w, std::max(1.0f, winH_ - kTopBar)};
}

bool DockLayout::hitSplitter(float mx, float my, DockSide& side) const {
    for (DockSide s : {DockSide::Left, DockSide::Right}) {
        RectF sp = splitter(s);
        if (sp.w <= 0.0f) continue;
        sp.x -= 3.0f;
        sp.w += 6.0f;
        if (sp.contains(mx, my)) {
            side = s;
            return true;
        }
    }
    return false;
}

void DockLayout::beginDrag(DockSide s, float mouseX) {
    if (!okNumber(mouseX) || collapsed(s)) return;
    dragging_ = true;
    dragSide_ = s;
    const RectF sp = splitter(s);
    dragGrabOffset_ = mouseX - (sp.x + sp.w * 0.5f);
}

void DockLayout::dragTo(float mouseX) {
    if (!dragging_ || !okNumber(mouseX)) return;
    const float edge = mouseX - dragGrabOffset_;   // onde deve ficar o centro da borda
    if (dragSide_ == DockSide::Left) setWidth(DockSide::Left, edge - kSplitter * 0.5f);
    else setWidth(DockSide::Right, winW_ - edge - kSplitter * 0.5f);
}

std::string DockLayout::serialize() const {
    char buf[160];
    std::snprintf(buf, sizeof buf, "viewer_layout 1\nleft %.0f\nright %.0f\nleft_collapsed %d\nright_collapsed %d\n",
                  static_cast<double>(leftW_), static_cast<double>(rightW_), leftCollapsed_ ? 1 : 0, rightCollapsed_ ? 1 : 0);
    return buf;
}

bool DockLayout::parse(const std::string& text) {
    float l = leftW_, r = rightW_;
    bool lc = leftCollapsed_, rc = rightCollapsed_;
    bool header = false;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        const std::size_t hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream ls(line);
        std::string key, val, extra;
        if (!(ls >> key)) continue;
        if (!(ls >> val) || (ls >> extra)) return false;
        char* end = nullptr;
        const double d = std::strtod(val.c_str(), &end);
        if (end != val.c_str() + val.size() || !std::isfinite(d)) return false;
        if (key == "viewer_layout") {
            if (d != 1.0) return false;
            header = true;
        } else if (key == "left" || key == "right") {
            if (d < 100.0 || d > 5000.0) return false;
            (key == "left" ? l : r) = static_cast<float>(d);
        } else if (key == "left_collapsed" || key == "right_collapsed") {
            if (d != 0.0 && d != 1.0) return false;
            (key == "left_collapsed" ? lc : rc) = d != 0.0;
        } else {
            return false;
        }
    }
    if (!header) return false;
    leftW_ = l;
    rightW_ = r;
    leftCollapsed_ = lc;
    rightCollapsed_ = rc;
    reclamp();
    return true;
}

// ----------------------------------------------------------------------------------------------------
// ScrollState
// ----------------------------------------------------------------------------------------------------
void ScrollState::setSizes(float contentH, float viewH) {
    contentH_ = okNumber(contentH) ? std::max(0.0f, contentH) : 0.0f;
    viewH_ = okNumber(viewH) ? std::max(0.0f, viewH) : 0.0f;
    setOffset(offset_);
}

void ScrollState::setOffset(float o) { offset_ = okNumber(o) ? clampf(o, 0.0f, maxOffset()) : offset_; }

float ScrollState::thumbHeight(float trackH) const {
    if (!scrollable() || trackH <= 0.0f) return std::max(0.0f, trackH);
    return clampf(trackH * viewH_ / contentH_, std::min(24.0f, trackH), trackH);
}

float ScrollState::thumbTop(float trackH) const {
    const float m = maxOffset();
    if (m <= 0.0f) return 0.0f;
    return (trackH - thumbHeight(trackH)) * (offset_ / m);
}

void ScrollState::dragThumb(float dy, float trackH) {
    const float range = trackH - thumbHeight(trackH);
    if (!okNumber(dy) || range <= 0.0f) return;
    setOffset(offset_ + dy * maxOffset() / range);
}

void ScrollState::jumpTo(float yInTrack, float trackH) {
    const float range = trackH - thumbHeight(trackH);
    if (!okNumber(yInTrack) || range <= 0.0f) return;
    setOffset((yInTrack - thumbHeight(trackH) * 0.5f) / range * maxOffset());
}

} // namespace viewer
