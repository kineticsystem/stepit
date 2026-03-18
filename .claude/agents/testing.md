# Testing Agent - Lentigram Test Best Practices

This agent provides testing strategies, patterns, and best practices specific to the Lentigram project.

## Test Output Directory Management

### CRITICAL RULE: Use Dynamic Test Names

**NEVER hardcode test suite names in output paths.** Always use Google Test's API to get test names dynamically.

#### For Test Fixtures (TEST_F with SetUp)

**Pattern:**
```cpp
class MyTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // ... other setup ...

        // Setup output directory for test artifacts
        const ::testing::TestInfo* test_info = ::testing::UnitTest::GetInstance()
                                                   ->current_test_info();
        QString testSuiteName = QString::fromUtf8(test_info->test_suite_name());
        m_outputDir = QString("%1/%2").arg(TEST_BASE_DIR, testSuiteName);
        QDir().mkpath(m_outputDir);
    }

    QString m_outputDir;
};

TEST_F(MyTest, SomeTest)
{
    const ::testing::TestInfo* test_info = ::testing::UnitTest::GetInstance()
                                               ->current_test_info();
    QString testName = QString::fromUtf8(test_info->name());

    QString outputPath = QString("%1/%2_result.png").arg(m_outputDir, testName);
    result.save(outputPath);
}
```

**Benefits:**
- Test suite directory created once in SetUp()
- Each test writes to its own subdirectory using test name
- Rename test → output path updates automatically
- No hardcoded strings to maintain

#### For Tests Without Fixtures (TEST)

**Pattern:**
```cpp
TEST(MyTest, SomeTest)
{
    const ::testing::TestInfo* test_info = ::testing::UnitTest::GetInstance()
                                               ->current_test_info();
    QString testSuiteName = QString::fromUtf8(test_info->test_suite_name());
    QString testName = QString::fromUtf8(test_info->name());

    QDir dir(QString("%1/%2").arg(TEST_BASE_DIR, testSuiteName));
    dir.mkpath(".");

    QString outputPath = QString("%1/%2_result.png").arg(dir.path(), testName);
    result.save(outputPath);
}
```

### Use Modern Google Test API

**✅ CORRECT:**
```cpp
QString testSuiteName = QString::fromUtf8(test_info->test_suite_name());
```

**❌ WRONG (deprecated):**
```cpp
QString testCaseName = test_info->test_case_name();  // Old API
```

### Multi-arg QString::arg()

**✅ CORRECT:**
```cpp
QString path = QString("%1/%2/%3").arg(base, suite, test);
```

**❌ WRONG (clazy warning):**
```cpp
QString path = QString("%1/%2/%3").arg(base).arg(suite).arg(test);
```

## Test Failure vs Skipping

### CRITICAL RULE: Fail Real Errors

**NEVER use GTEST_SKIP() for initialization failures.** Real errors must fail tests, not skip them.

**✅ CORRECT:**
```cpp
QString modelPath = ResourceFinder::find("model.onnx");
ASSERT_FALSE(modelPath.isEmpty()) << "Model not found: model.onnx";

auto result = estimator.initialize(modelPath);
ASSERT_TRUE(result) << "Failed to initialize: " << result.error().toStdString();
```

**❌ WRONG:**
```cpp
if (modelPath.isEmpty()) {
    GTEST_SKIP() << "Model not found";  // Hides real failures!
}
```

**Why:** CI/CD must catch missing resources and initialization failures. Skipped tests hide problems.

### When to Use ASSERT vs EXPECT

**Use ASSERT when:**
- Failure makes the rest of the test meaningless
- Example: File not loaded → can't test processing

**Use EXPECT when:**
- Test should continue checking other conditions
- Example: Multiple pixel checks, statistics validation

## Image Processing Tests

### Format Preservation

When testing image processors, **verify format preservation:**

```cpp
TEST_F(ProcessorTest, FormatPreservation)
{
    QImage input = loadImage();  // e.g., RGB888
    QImage result = processor.process(input);

    // Verify output format matches input format
    EXPECT_EQ(result.format(), input.format());
}
```

### Zero-Effect Operations

When testing zero-effect operations (blur=0, scale=1.0), **verify pixel-perfect equality:**

```cpp
TEST_F(ProcessorTest, ZeroBlurNoChange)
{
    QImage input = loadImage();
    QImage result = processor.blur(input, /*blurRadius=*/0.0f);

    ASSERT_EQ(input.size(), result.size());
    EXPECT_EQ(input.format(), result.format());

    // Pixel-perfect comparison
    int differingPixels = 0;
    for (int y = 0; y < input.height(); ++y) {
        for (int x = 0; x < input.width(); ++x) {
            if (input.pixel(x, y) != result.pixel(x, y)) {
                ++differingPixels;
            }
        }
    }

    EXPECT_EQ(0, differingPixels) << "Images are not identical";
}
```

**Don't allow tolerance for operations that should be exact.**

### Avoid Unnecessary Format Conversions

**In production code:**
```cpp
QImage process(const QImage& image)
{
    QImage::Format originalFormat = image.format();

    // Convert to working format only if necessary
    QImage working = (image.format() == QImage::Format_ARGB32)
                         ? image
                         : image.convertToFormat(QImage::Format_ARGB32);

    // ... process working ...

    // Convert back only if necessary
    if (result.format() != originalFormat) {
        return result.convertToFormat(originalFormat);
    }
    return result;
}
```

## Test Organization

### Directory Structure

```
TEST_BASE_DIR/
├── TestSuiteName/
│   ├── TestName1_output.png
│   ├── TestName2_output.png
│   └── subdirs_if_needed/
└── AnotherTestSuite/
    └── ...
```

**Achieved automatically with dynamic test names.**

### Test Naming

**Use descriptive test names that explain:**
- What is being tested
- Under what conditions
- Expected behavior

**✅ GOOD:**
```cpp
TEST_F(DepthEstimatorTest, EstimateDepth_WithoutInitialization_FailsGracefully)
TEST_F(BokehProcessorTest, ZeroBlurRadiusNoChange)
TEST_F(ImageProcessorTest, FormatPreservation_RGB888Input)
```

**❌ BAD:**
```cpp
TEST_F(DepthEstimatorTest, DifferentImageSizes)  // Misleading - actually tests uninitialized behavior
TEST_F(BokehProcessorTest, Test1)
```

## Parameterized Tests

### CRITICAL: Test Names Contain Slashes

**Parameterized test names include the parameter index with a forward slash:**
- `TestName/0`
- `TestName/1`
- `TestName/2`

**You MUST sanitize test names before using in file paths:**

```cpp
const ::testing::TestInfo* test_info = ::testing::UnitTest::GetInstance()
                                           ->current_test_info();
QString testName = QString::fromUtf8(test_info->name());
testName.replace('/', '_');  // CRITICAL: Replace slashes for file paths

QString filename = QString("%1/%2_result.png").arg(m_outputDir, testName);
```

**Without this, file saves will fail silently!**

### Settings-Based Parameterization

**Pattern for testing multiple configurations:**

```cpp
// Helper to create settings (if no constructor available)
namespace {
Settings createSettings(float param1, float param2)
{
    Settings s;
    s.param1 = param1;
    s.param2 = param2;
    return s;
}
}

class MyTest : public ::testing::TestWithParam<Settings>
{
protected:
    void SetUp() override
    {
        // ... setup m_outputDir as shown above ...
    }

    QString m_outputDir;
};

INSTANTIATE_TEST_SUITE_P(
    VariousConfigurations,
    MyTest,
    ::testing::Values(
        createSettings(1.0f, 2.0f),
        createSettings(3.0f, 4.0f),
        createSettings(5.0f, 6.0f)
    ));

TEST_P(MyTest, SomeTest)
{
    Settings settings = GetParam();
    // ... use settings ...
}
```

## Resource Finding

### ALWAYS Use ResourceFinder

**✅ CORRECT:**
```cpp
QString modelPath = common::ResourceFinder::find("model.onnx");
ASSERT_FALSE(modelPath.isEmpty()) << "Model not found: model.onnx";
```

**❌ WRONG:**
```cpp
QStringList possiblePaths = {
    "../../../bin/model.onnx",
    "./model.onnx",
    "../bin/model.onnx"
};
// ... manual search ...
```

**Why:** ResourceFinder handles all platforms and deployment scenarios (AppImage, normal builds, etc.)

## Common Patterns

### Loading Test Images from Resources

```cpp
void SetUp() override
{
    QString imagePath = ":/resources/test_image.jpg";
    m_testImage = QImage(imagePath);
    ASSERT_FALSE(m_testImage.isNull()) << "Test image not found: test_image.jpg";

    // Setup output directory
    const ::testing::TestInfo* test_info = ::testing::UnitTest::GetInstance()
                                               ->current_test_info();
    QString testSuiteName = QString::fromUtf8(test_info->test_suite_name());
    m_outputDir = QString("%1/%2").arg(TEST_BASE_DIR, testSuiteName);
    QDir().mkpath(m_outputDir);
}
```

### Testing with Temporary Files

```cpp
TEST_F(MyTest, FileOperation)
{
    QTemporaryDir tempDir;
    ASSERT_TRUE(tempDir.isValid());

    QString testFile = tempDir.filePath("test.dat");
    // ... test file operations ...
}
```

## Integration Tests

### Test Fixtures with Heavy Setup

For tests requiring model loading, rendering, or other expensive operations:

```cpp
class HeavyTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Find model
        QString modelPath = ResourceFinder::find("model.onnx");
        ASSERT_FALSE(modelPath.isEmpty()) << "Model not found";

        // Initialize (expensive operation)
        auto result = m_processor.initialize(modelPath);
        ASSERT_TRUE(result) << "Initialization failed: " << result.error().toStdString();

        // Setup output directory
        const ::testing::TestInfo* test_info = ::testing::UnitTest::GetInstance()
                                                   ->current_test_info();
        QString testSuiteName = QString::fromUtf8(test_info->test_suite_name());
        m_outputDir = QString("%1/%2").arg(TEST_BASE_DIR, testSuiteName);
        QDir().mkpath(m_outputDir);
    }

    Processor m_processor;
    QString m_outputDir;
};
```

**Key Point:** Expensive operations in SetUp() run once per test, not per test suite. If you need shared setup across all tests, consider `SetUpTestSuite()` (static method).

## Checklist for New Tests

- [ ] Uses dynamic test names (no hardcoded paths)
- [ ] Uses `test_suite_name()` not `test_case_name()`
- [ ] Uses multi-arg `QString::arg()`
- [ ] Uses `ASSERT_FALSE(path.isEmpty())` not `GTEST_SKIP()`
- [ ] Verifies format preservation for image tests
- [ ] Uses pixel-perfect comparison for zero-effect operations
- [ ] Uses ResourceFinder for finding models/resources
- [ ] Has descriptive test name explaining what/how/expected
- [ ] Creates output directory in SetUp() for fixtures
- [ ] Uses ASSERT for critical failures, EXPECT for checks

## Anti-Patterns to Avoid

❌ Hardcoded test suite names in paths
❌ Using deprecated `test_case_name()`
❌ Chaining `.arg().arg().arg()`
❌ Using `GTEST_SKIP()` for real failures
❌ Allowing tolerance for exact operations
❌ Unnecessary format conversions
❌ Manual resource path searching
❌ Generic test names like "Test1", "DifferentSizes"
❌ Not checking format preservation
❌ Comparing images without size/format checks first

---

**Remember:** Tests are documentation. Make them clear, maintainable, and self-describing.
