# Development Guidelines

This document outlines the development tools and processes for the Liquid Engine Flight Software project.

## Code Formatting

We use `clang-format` with a custom configuration to ensure consistent code style across the project.

### Format Script

The `./format.sh` script handles all code formatting:

```bash
# Check formatting without making changes
./format.sh --check

# Format all code
./format.sh

# Format with verbose output
./format.sh --verbose

# Format all files including external dependencies
./format.sh --all

# Show help
./format.sh --help
```

### Formatting Configuration

The `.clang-format` file defines our formatting standards:
- Based on Google C++ style
- 4-space indentation
- 100 character line limit
- Aerospace-specific customizations

### Pre-commit Formatting

Always format your code before committing:
```bash
./format.sh
git add .
git commit -m "Your commit message"
```

## CI/CD Pipeline

GitHub Actions automatically runs on every push and pull request:

### Workflows

1. **Build and Test** (`build-and-test` job):
   - Builds in both Debug and Release modes
   - Runs static analysis with `cppcheck`
   - Checks code formatting
   - Runs tests (when available)
   - Uploads build artifacts for Release builds

2. **Code Quality** (`code-quality` job):
   - Validates code formatting
   - Checks for TODO/FIXME/HACK comments

3. **Security Scan** (`security-scan` job):
   - Runs security analysis with `semgrep`

### Required Dependencies

The CI pipeline installs:
- `build-essential`, `cmake`
- `libeigen3-dev`, `pkg-config`
- `clang-format`, `cppcheck`
- `libcanard-dev`, `valgrind`

### Local Development

To run the same checks locally:

```bash
# Install dependencies (Ubuntu/Debian)
sudo apt-get install build-essential cmake libeigen3-dev clang-format cppcheck pkg-config libcanard-dev valgrind

# Build project
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Run static analysis
cppcheck --enable=all --inconclusive --std=c++17 \
  --suppress=missingIncludeSystem \
  --suppress=unusedFunction \
  --suppress=noExplicitConstructor \
  FSW/ comms/ utl/

# Check formatting
./format.sh --check
```

## Code Standards

### C++ Standard
- **C++17** (defined in `CMakeLists.txt`)

### Style Guidelines
- Follow the `.clang-format` configuration
- Use meaningful variable and function names
- Add comments for complex algorithms
- Keep functions focused and small
- Use `const` where appropriate

### File Organization
- Header files: `.hpp`
- Source files: `.cpp`
- Include guards or `#pragma once`
- Organized in logical directories

### Error Handling
- Use exceptions for exceptional cases
- Return error codes for expected failures
- Log errors appropriately

## Building the Project

```bash
# Clean build
rm -rf build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Install
sudo make install
```

## Testing

Currently, the project structure supports unit tests, but they need to be implemented. When adding tests:

1. Create test files in `tests/` directory
2. Update `CMakeLists.txt` to include test targets
3. Tests will automatically run in CI

## Contributing

1. Create a feature branch
2. Make your changes
3. Run `./format.sh` to format code
4. Test your changes locally
5. Push to your branch
6. Create a pull request

The CI pipeline will automatically validate your changes.
