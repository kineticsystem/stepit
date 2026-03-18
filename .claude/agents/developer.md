# Lentigram Developer Agent

This agent provides deep knowledge of Lentigram's architecture, design patterns, and best practices. Use this agent when working on the Lentigram codebase.

## Critical Architectural Principles

### Model as Source of Truth

**RULE**: Data models are the single source of truth and the communication bridge between components.

**Key Points**:
- **Never cache model data** in actions, plugins, or mediators
- **Read from model** when you need data, don't store copies
- **Write to model** when data changes, don't sync copies with signals
- **Models are bridges**: Components communicate state through models, not via direct signals

**Anti-Pattern** (Before):
```cpp
// BAD - Cached state synced via signals
class PrecomputeAction {
    QPointF m_cachedPoint;
    float m_cachedRadius;
    void setParameters(QPointF p, float r) { m_cachedPoint = p; m_cachedRadius = r; }
};
// Need signals: plugin->focusChanged → action->setParameters()
```

**Correct Pattern** (After):
```cpp
// GOOD - Read from model when needed
class PrecomputeAction {
    std::shared_ptr<ProjectData> m_project;
    void execute() {
        auto bokeh = m_project->bokeh();
        float radius = bokeh->blurAmount().toFloat();  // Read when needed
        QPointF point = bokeh->focusPoint().toQPointF();
    }
};
// No signals needed - model is the source of truth
```

**Benefits**:
- ✅ No synchronization bugs (can't get out of sync)
- ✅ Single source of truth (one place to look)
- ✅ Simpler code (no signal connections for sync)
- ✅ Always fresh data (read current value, not stale cache)

**Test**: If you need signals to sync state between components, you're doing it wrong - use the model instead.

---

### Error Handling with std::expected

**RULE**: Use `std::expected<T, E>` (C++23) for error handling instead of storing errors in member variables or using output parameters.

**Key Points**:
- **Prefer `std::expected`** for all functions that can fail
- **Return errors directly** - no need for separate `lastError()` methods
- **Cleaner API**: Error handling is explicit and forced by the type system
- **No state pollution**: No need for mutable error member variables

**Anti-Pattern** (Before):
```cpp
// BAD - Error stored in member variable
class DepthEstimator {
    QString m_lastError;
    mutable QMutex m_errorMutex;
public:
    bool initialize(const QString& path);
    QString lastError() const;  // Separate method to get error
};

// Usage requires two steps:
if (!estimator.initialize(path)) {
    qWarning() << "Error:" << estimator.lastError();  // Extra call needed
}
```

**Correct Pattern** (After):
```cpp
// GOOD - Error returned directly
class DepthEstimator {
public:
    std::expected<void, QString> initialize(const QString& path);
};

// Usage is cleaner and more explicit:
auto result = estimator.initialize(path);
if (!result) {
    qWarning() << "Error:" << result.error();  // Error is directly available
}
```

**Benefits**:
- ✅ Type-safe error handling (compiler forces you to check)
- ✅ No state pollution (no error member variables)
- ✅ Cleaner API (one method, not two)
- ✅ Thread-safe by design (no shared mutable state)
- ✅ Composable (can use monadic operations)

**When to use**:
- Functions that return a value and can fail: `std::expected<T, QString>`
- Functions that return void and can fail: `std::expected<void, QString>`
- Error type is usually `QString` for descriptive messages

**Example from codebase**:
```cpp
// DepthEstimator::initialize
std::expected<void, QString> initialize(const QString& modelPath, int numThreads = 0);

// DepthEstimator::estimateDepth
std::expected<QImage, QString> estimateDepth(const QImage& input);

// Usage:
auto initResult = estimator.initialize(modelPath);
if (!initResult) {
    return initResult.error();  // Propagate error
}

auto depthResult = estimator.estimateDepth(image);
if (depthResult) {
    QImage depth = depthResult.value();
    // Use depth map...
}
```

---

## Design Patterns

### Action Pattern
User actions encapsulated in action classes (src/lentigram-lib/src/actions/)
- Actions coordinate between models, workers, and UI
- Actions manage async operations and progress reporting
- Actions read from models when executing, don't cache state

### Command Pattern
Undoable operations (src/lentigram-lib/src/commands/)
- Commands encapsulate state changes
- Support undo/redo via QUndoStack

### MVVM Pattern
ViewModels bind data models to Qt widgets (src/lentigram-lib/src/viewmodels/)
- ViewModels expose model data to UI
- ViewModels handle UI-specific logic

### Singleton Pattern with std::shared_ptr

**RULE**: Singletons should return `std::shared_ptr` instead of raw pointers for better safety.

**Anti-Pattern** (Before):
```cpp
// BAD - Raw pointer with unclear ownership
class MySingleton {
public:
    static MySingleton* instance();
private:
    ~MySingleton() = default;  // Private destructor
};

// Caller has unclear ownership
auto* singleton = MySingleton::instance();  // Can I delete this?
```

**Correct Pattern** (After):
```cpp
// GOOD - Shared pointer with clear ownership
class MySingleton {
public:
    static std::shared_ptr<MySingleton> instance();
private:
    ~MySingleton() = default;  // Private destructor
};

// Implementation with no-op deleter (singleton lives forever)
std::shared_ptr<MySingleton> MySingleton::instance() {
    static std::shared_ptr<MySingleton> instance(
        new MySingleton(),
        [](MySingleton*) { /* No-op: singleton lives forever */ }
    );
    return instance;
}

// Caller has clear ownership semantics
auto singleton = MySingleton::instance();  // Shared ownership, can't accidentally delete
```

**Benefits**:
- ✅ Clear ownership semantics (shared, not owned by caller)
- ✅ Can't accidentally delete the singleton
- ✅ Type-safe (no raw pointer conversions)
- ✅ Consistent with modern C++ idioms

**Example from codebase**: `DepthEstimatorService::instance()`

---

## Performance Best Practices

### Use scanLine() for Pixel Access

**Wrong (10-50× slower)**:
```cpp
for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
        QRgb p = image.pixel(x, y);  // Slow method call
    }
}
```

**Correct (fast)**:
```cpp
for (int y = 0; y < height; ++y) {
    const QRgb *line = reinterpret_cast<const QRgb *>(image.constScanLine(y));
    for (int x = 0; x < width; ++x) {
        QRgb p = line[x];  // Direct memory access
    }
}
```

**Why reinterpret_cast is safe here**:
1. We guarantee Format_ARGB32 format (each pixel is exactly 32 bits)
2. QRgb is typedef for uint32_t (also exactly 32 bits)
3. Memory layout matches perfectly: uchar[4] = 1 QRgb
4. This is standard Qt practice throughout Qt's own image processing code

### Thread Safety Patterns

**Status Updates with Task Cancellation**:
```cpp
// In action constructor:
m_statusConnection = connect(m_statusController.get(), &StatusController::status,
    this, [this, currentTaskId](const Status &status) {
        if (currentTaskId == m_taskId.load()) {  // Check if still current
            Q_EMIT statusUpdate(status);
        }
    }, Qt::QueuedConnection);

// In cancelProcessing():
if (m_statusConnection) {
    disconnect(m_statusConnection);
}
```

**Atomic Task ID for Cancellation**:
```cpp
std::atomic<uint64_t> m_taskId{0};

void processFocusBokeh(...) {
    uint64_t currentTaskId = ++m_taskId;
    // ... later in callbacks:
    if (taskId != m_taskId.load()) {
        return;  // Task was superseded
    }
}
```

### Parallel Processing with Thread Limits

**Problem**: Using all CPU cores starves the UI thread

**Solution**: Limit to half available cores
```cpp
constexpr int THREAD_POOL_DIVISOR = 2;  // Named constant, not magic number

QThreadPool limitedPool;
int maxThreads = qMax(1, QThread::idealThreadCount() / THREAD_POOL_DIVISOR);
limitedPool.setMaxThreadCount(maxThreads);
```

### Image Processing Best Practices

1. **Always process at full resolution** for maximum quality
2. **Only scale down for storage/display** (preview + thumbnail)
3. **Use Qt's format conversions**:
   ```cpp
   QImage result = image.convertToFormat(QImage::Format_ARGB32);  // For QRgb operations
   QImage depth = depthMap.convertToFormat(QImage::Format_Grayscale8);  // For single-channel
   ```
4. **Lossless debug saves**: Use PNG for debugging, not JPEG
5. **Use percentage-based values** for effects (not absolute pixels) for consistent results across image sizes

---

## Qt Best Practices for Lentigram

### Always Use Braces for `if` Bodies

**RULE**: Always wrap `if` (and `else`) bodies in braces, even for single-line statements.

**Wrong**:
```cpp
if (originalImage.isNull())
    return std::unexpected(loadError.arg(file->filePath()));

if (!result)
    return result.error();
```

**Correct**:
```cpp
if (originalImage.isNull()) {
    return std::unexpected(loadError.arg(file->filePath()));
}

if (!result) {
    return result.error();
}
```

**Reason**: Prevents subtle bugs when lines are added later, and keeps style consistent throughout the codebase.

---

### Use Q_UNUSED, Not [[maybe_unused]]

**Wrong**:
```cpp
[[maybe_unused]] auto future = QtConcurrent::run(...);
```

**Correct**:
```cpp
auto future = QtConcurrent::run(...);
Q_UNUSED(future);
```

**Reason**: Project convention, consistent with Qt codebase.

### Named Constants, Not Magic Numbers

**Wrong**:
```cpp
int maxThreads = qMax(1, QThread::idealThreadCount() / 2);  // Magic 2
```

**Correct**:
```cpp
constexpr int THREAD_POOL_DIVISOR = 2;  // Named, documented
int maxThreads = qMax(1, QThread::idealThreadCount() / THREAD_POOL_DIVISOR);
```

---

## Common Pitfalls

1. **Using pixel() in loops**: 10-50× slower than scanLine()
2. **Blocking wait loops**: Use futures properly, don't msleep()
3. **Slow operations on UI thread**: Move ALL slow ops to background
4. **Using all CPU cores**: Limit threads to avoid UI starvation
5. **Complex algorithms**: Simpler is often faster and more maintainable
6. **Single-pixel sampling**: Average a window for noise reduction
7. **Forgetting task cancellation**: Always check if task is still current
8. **Using [[maybe_unused]]**: Use Q_UNUSED instead (Qt convention)
9. **Magic numbers**: Use named constants with documentation
10. **O(n) when O(1) possible**: Calculate directly instead of searching
11. **Absolute pixel values for effects**: Use percentage-based values for consistent results
12. **Caching model state**: Read from model when needed, don't cache
13. **Depth convention mismatches**: Document conventions clearly (0=near vs 1=near)

---

## Depth-lib Specific Knowledge

### Depth Convention
- **Depth map values**: 1.0 = near/white, 0.0 = far/black
- **BokehProcessor API**: 0.0 = near/white, 1.0 = far/black (inverted!)
- **Always invert when passing to BokehProcessor**: `settings.focusDepth = 1.0f - depthMapValue`

### Blur Pyramid Approach
Instead of computing blur per-pixel (slow), pre-compute N blur levels from 0% to 100%, then blend based on depth.

**Levels**:
- Level 0: No blur (original image)
- Level N-1: 100% of max blur
- Blend between levels using depth difference

**Background-Only Blur**:
```cpp
float depthDiff = std::max(0.0f, focusDepth - d);
float requiredBlur = depthDiff * maxBlurRadius;
```
Only blur objects farther than focus point.

### StackBlur Algorithm
- **Complexity**: O(w×h) regardless of radius (10-20× faster than Gaussian)
- **Implementation**: Sliding window with running sums
- **Uses reinterpret_cast**: Safe because Format_ARGB32 is guaranteed

---

## Code Review Checklist

- [ ] Braces on every `if`/`else` body, even single-line
- [ ] No cached model state (read from model instead)
- [ ] Use scanLine() for pixel access (not pixel/setPixel)
- [ ] Named constants instead of magic numbers
- [ ] Q_UNUSED instead of [[maybe_unused]]
- [ ] Thread pool limits for background processing
- [ ] Proper task cancellation with atomic task IDs
- [ ] Percentage-based values for image effects
- [ ] Process at full resolution, scale down for storage
- [ ] Status updates with Qt::QueuedConnection
- [ ] All slow operations in background threads

---

## Key Dependencies

### Internal Libraries
- **common-lib**: Shared utilities (math, GUI helpers, geometry, StatusController)
- **data-lib**: Data models (ProjectData, BokehData, FileData, etc.)
- **depth-lib**: Depth estimation (DepthEstimator, BokehProcessor, StackBlur)
- **interlacer-lib**: Core interlacing algorithm

### External Libraries
- **Qt6**: Core, Gui, Widgets, OpenGL, Concurrent, Network
- **ONNX Runtime**: Depth estimation inference
- **Cycles**: Ray-tracing render engine
- **Boost, GLM, TBB**: Utilities and threading

---

## Testing Guidelines

### Image Processing Features Checklist
- [ ] First run (generates depth map): ~3-9 seconds
- [ ] Second run (cached depth): ~2-6 seconds, noticeably faster
- [ ] Effect at minimum: No visible change
- [ ] Effect at maximum: Strong visible effect
- [ ] Interruption: Cancel old task, start new task
- [ ] No crashes or "unresponsive" warnings
- [ ] File switching: Display updates correctly
- [ ] Tool switching: Returns to correct display mode
- [ ] Quality: Process at full resolution

---

## Build Commands Quick Reference

```bash
# Install dependencies
./install-deps.sh

# Configure (requires Qt6_DIR set)
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug

# Build
cmake --build build -j8

# Run all tests
ctest --test-dir build -V

# Run specific library tests
ctest --test-dir build/src/depth-lib/test -V

# Run application
./build/bin/lentigram
```

---

## When to Use This Agent

Invoke this agent when:
- Starting work on a new feature or refactoring
- Reviewing code for best practices
- Debugging performance issues
- Making architectural decisions
- Setting up new actions, commands, or viewmodels
- Working with depth-lib or image processing
- Unsure about Qt patterns or threading

This agent complements CLAUDE.md (project overview) by providing deep implementation knowledge and lessons learned.
