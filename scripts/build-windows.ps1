param(
    [string]$BuildDir,
    [string]$OutDir,
    [string]$DependencyDirectory,
    [string]$SourceDirectory,
    [string]$BulletArchive,
    [switch]$Test,
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Release'
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
if (!$BuildDir) { $BuildDir = Join-Path $repo 'build-windows-native' }
$BuildDir = [IO.Path]::GetFullPath($BuildDir)
if (!$OutDir) {
    $distribution = if ($Configuration -eq 'Debug') { 'dist-debug' } else { 'dist' }
    $OutDir = Join-Path $BuildDir $distribution
}
$OutDir = [IO.Path]::GetFullPath($OutDir)
if (!$DependencyDirectory) { $DependencyDirectory = Join-Path $BuildDir 'dependencies' }
$DependencyDirectory = [IO.Path]::GetFullPath($DependencyDirectory)
if (!$SourceDirectory) { $SourceDirectory = Join-Path $DependencyDirectory 'sources' }
$SourceDirectory = [IO.Path]::GetFullPath($SourceDirectory)
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (!(Test-Path -LiteralPath $vswhere)) { throw 'Install Visual Studio with the Desktop development with C++ workload.' }
$vs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -format json | ConvertFrom-Json
if (!$vs) { throw 'Visual Studio C++ tools were not found.' }
$root = @($vs)[0].installationPath
$version = [Version]@($vs)[0].installationVersion
if ($version.Major -eq 17) { $generator = 'Visual Studio 17 2022' }
elseif ($version.Major -eq 18) { $generator = 'Visual Studio 18 2026' }
else { throw "Unsupported Visual Studio version $version." }
$cmakeRoot = Join-Path $root 'Common7\IDE\CommonExtensions\Microsoft\CMake'
$cmake = Join-Path $cmakeRoot 'CMake\bin\cmake.exe'
$ctest = Join-Path $cmakeRoot 'CMake\bin\ctest.exe'
if (!(Test-Path -LiteralPath $cmake)) { throw 'Install the C++ CMake tools for Windows component in Visual Studio.' }
$jobs = [Math]::Min([Environment]::ProcessorCount, 8)
$visualStudio = @('-G', $generator, '-A', 'x64', '-DCMAKE_POLICY_DEFAULT_CMP0091=NEW')

function Assert-NativeSuccess([string]$Operation) {
    if ($LASTEXITCODE -ne 0) { throw "$Operation failed ($LASTEXITCODE)." }
}

function Build-Dependency([string]$Name, [string]$Source, [string]$Url, [string]$Hash,
                          [string]$ReleaseFile, [string]$DebugFile, [string[]]$Flags) {
    $sourcePath = Join-Path $SourceDirectory $Source
    if (!(Test-Path -LiteralPath (Join-Path $sourcePath 'CMakeLists.txt'))) {
        New-Item -ItemType Directory -Force -Path $SourceDirectory | Out-Null
        $tar = Join-Path $SourceDirectory "$Source.tar.gz"
        if (!(Test-Path -LiteralPath $tar) -or (Get-FileHash -LiteralPath $tar).Hash -ne $Hash) {
            curl.exe --fail --location --silent --show-error --retry 2 --connect-timeout 20 --output $tar $Url
            Assert-NativeSuccess "Download $Source"
        }
        if ((Get-FileHash -LiteralPath $tar).Hash -ne $Hash) { throw "SHA-256 mismatch for $Source." }
        Push-Location $SourceDirectory
        try {
            & $cmake -E tar xzf $tar
            Assert-NativeSuccess "Extract $Source"
        } finally { Pop-Location }
    }
    $install = Join-Path $DependencyDirectory $Name
    foreach ($config in @('Release', 'Debug')) {
        if ($config -eq 'Debug') {
            $installedFile = $DebugFile
            $obj = Join-Path $DependencyDirectory "$Name-debug-build"
            $runtime = 'MultiThreadedDebug'
        } else {
            $installedFile = $ReleaseFile
            $obj = Join-Path $DependencyDirectory "$Name-build"
            $runtime = 'MultiThreaded'
        }
        if (Test-Path -LiteralPath (Join-Path $install $installedFile)) { continue }
        & $cmake -S $sourcePath -B $obj @visualStudio "-DCMAKE_CONFIGURATION_TYPES=$config" `
            "-DCMAKE_MSVC_RUNTIME_LIBRARY=$runtime" @Flags
        Assert-NativeSuccess "Configure $Name ($config)"
        & $cmake --build $obj --config $config --parallel $jobs
        Assert-NativeSuccess "Build $Name ($config)"
        & $cmake --install $obj --config $config --prefix $install
        Assert-NativeSuccess "Install $Name ($config)"
    }
}

$imath = Join-Path $DependencyDirectory 'imath'
$tbb = Join-Path $DependencyDirectory 'tbb'
$vdb = Join-Path $DependencyDirectory 'openvdb'
Build-Dependency 'imath' 'Imath-3.2.1' `
    'https://codeload.github.com/AcademySoftwareFoundation/Imath/tar.gz/refs/tags/v3.2.1' `
    'b2c8a44c3e4695b74e9644c76f5f5480767355c6f98cde58ba0e82b4ad8c63ce' `
    'lib/cmake/Imath/ImathTargets-release.cmake' 'lib/cmake/Imath/ImathTargets-debug.cmake' `
    @('-DBUILD_SHARED_LIBS=OFF','-DBUILD_TESTING=OFF')
Build-Dependency 'tbb' 'oneTBB-2022.3.0' `
    'https://codeload.github.com/uxlfoundation/oneTBB/tar.gz/refs/tags/v2022.3.0' `
    '01598a46c1162c27253a0de0236f520fd8ee8166e9ebb84a4243574f88e6e50a' `
    'lib/cmake/TBB/TBBTargets-release.cmake' 'lib/cmake/TBB/TBBTargets-debug.cmake' `
    @('-DBUILD_SHARED_LIBS=OFF','-DTBB_TEST=OFF','-DTBB_EXAMPLES=OFF',
                                    '-DTBBMALLOC_BUILD=OFF','-DTBBMALLOC_PROXY_BUILD=OFF','-DTBB_STRICT=OFF')
Build-Dependency 'openvdb' 'openvdb-13.0.0' `
    'https://codeload.github.com/AcademySoftwareFoundation/openvdb/tar.gz/refs/tags/v13.0.0' `
    '4d6a91df5f347017496fe8d22c3dbb7c4b5d7289499d4eb4d53dd2c75bb454e1' `
    'lib/libopenvdb.lib' 'lib/libopenvdb_d.lib' `
    @("-DCMAKE_PREFIX_PATH=$imath;$tbb",'-DOPENVDB_BUILD_BINARIES=OFF','-DCMAKE_DEBUG_POSTFIX=_d',
                       '-DOPENVDB_CORE_SHARED=OFF','-DOPENVDB_CORE_STATIC=ON','-DOPENVDB_USE_DELAYED_LOADING=OFF',
                       '-DUSE_BLOSC=OFF','-DUSE_ZLIB=OFF','-DUSE_IMATH_HALF=ON','-DUSE_EXPLICIT_INSTANTIATION=OFF')

if ($BulletArchive) { $BulletArchive = [IO.Path]::GetFullPath($BulletArchive) }
& $cmake -S $repo -B $BuildDir @visualStudio '-DCMAKE_CONFIGURATION_TYPES=Debug;Release' `
    '-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded$<$<CONFIG:Debug>:Debug>' `
    "-DCMAKE_PREFIX_PATH=$vdb;$tbb;$imath" '-DOPENVDB_USE_STATIC_LIBS=ON' `
    "-DTBB_DIR=$(Join-Path $tbb 'lib/cmake/TBB')" "-DImath_DIR=$(Join-Path $imath 'lib/cmake/Imath')" `
    "-DOPENVDB_MODULE_DIR=$(Join-Path $vdb 'lib/cmake/OpenVDB')" `
    "-DOpenVDB_INCLUDE_DIR=$(Join-Path $vdb 'include')" "-DOpenVDB_openvdb_INCLUDE_DIR=$(Join-Path $vdb 'include')" `
    "-DOpenVDB_openvdb_LIBRARY=$(Join-Path $vdb 'lib/libopenvdb.lib')" `
    "-DVOXEL_OPENVDB_DEBUG_LIBRARY=$(Join-Path $vdb 'lib/libopenvdb_d.lib')" "-DVOXEL_BULLET_ARCHIVE=$BulletArchive"
Assert-NativeSuccess 'Configure VoxelWorld'
& $cmake --build $BuildDir --config $Configuration --parallel $jobs
Assert-NativeSuccess "Build VoxelWorld ($Configuration)"
& $cmake --install $BuildDir --config $Configuration --prefix $OutDir
Assert-NativeSuccess "Install VoxelWorld ($Configuration)"
Copy-Item -LiteralPath (Join-Path $SourceDirectory 'openvdb-13.0.0/LICENSE') -Destination (Join-Path $OutDir 'licenses/OpenVDB-LICENSE.txt')
Copy-Item -LiteralPath (Join-Path $SourceDirectory 'oneTBB-2022.3.0/LICENSE.txt') -Destination (Join-Path $OutDir 'licenses/oneTBB-LICENSE.txt')
Copy-Item -LiteralPath (Join-Path $SourceDirectory 'Imath-3.2.1/LICENSE.md') -Destination (Join-Path $OutDir 'licenses/Imath-LICENSE.md')
Write-Output "Viewer: $(Join-Path $OutDir 'VoxelWorldViewer.exe')"

if ($Test) {
    Push-Location $BuildDir
    try {
        & $ctest -C $Configuration --output-on-failure
        Assert-NativeSuccess 'CTest'
    } finally { Pop-Location }
}
