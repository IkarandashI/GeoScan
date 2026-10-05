#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Systems/BenchmarkRunner.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBenchmarkSmokeTest, "GeoScan.Benchmark.Smoke",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FBenchmarkSmokeTest::RunTest(const FString&)
{
	const FString CsvPath = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Tests"), TEXT("BenchmarkSmoke.csv"));

	const TArray<FBenchmarkRow> Rows = Benchmark::Run(2, 500, CsvPath);
	TestEqual("Two seeds x three modes", Rows.Num(), 6);

	int32 Planned = 0;
	int32 Flown = 0;
	for (const FBenchmarkRow& Row : Rows)
	{
		Planned += Row.bPlanSuccess ? 1 : 0;
		Flown += Row.bPlaybackSuccess ? 1 : 0;
	}
	TestEqual("Every combination planned", Planned, 6);
	TestEqual("Every playback reached the boundary", Flown, 6);

	TestTrue("CSV report written", IFileManager::Get().FileExists(*CsvPath));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
