# CI/CD Pipeline Documentation

## Overview

Jessy uses GitHub Actions for continuous integration and deployment. The pipeline ensures code quality, runs comprehensive tests, performs security scans, and automates deployments.

## Workflows

### CI Workflow (`.github/workflows/ci.yml`)

Runs on every push and pull request to `main` and `develop` branches.

#### Jobs

**1. Lint and Format**
- Checks Rust code formatting with `rustfmt`
- Runs `clippy` linter with warnings as errors
- Checks Go code formatting with `gofmt`
- Uses cargo caching for faster builds

**2. Test Suite**
- Builds test Docker image
- Runs unit tests in container
- Runs integration tests with services
- Runs BDD tests with Cucumber
- Generates coverage report
- Uploads coverage to Codecov
- Fails if coverage < 80%

**3. Build Docker Images** (main branch only)
- Builds production Rust and Go images
- Tags with commit SHA and `latest`
- Pushes to Docker Hub
- Uses BuildKit caching for speed

**4. Security Scanning**
- Scans filesystem with Trivy
- Runs `cargo audit` for vulnerabilities
- Scans Docker images
- Uploads results to GitHub Security
- Fails on critical vulnerabilities

**5. Performance Benchmarks** (pull requests only)
- Runs benchmarks on base branch
- Runs benchmarks on PR branch
- Compares performance
- Comments results on PR

### Deploy Workflow (`.github/workflows/deploy.yml`)

Handles deployments to staging and production environments.

#### Staging Deployment
- **Trigger**: Push to `main` branch or manual dispatch
- **Steps**:
  1. Build and push Docker images
  2. Deploy to staging environment
  3. Run smoke tests
  4. Notify deployment status

#### Production Deployment
- **Trigger**: Version tags (`v*`) or manual dispatch with approval
- **Steps**:
  1. Verify tag format (v1.2.3)
  2. Build and push versioned images
  3. Deploy to production
  4. Run production smoke tests
  5. Create GitHub release
  6. Notify deployment status

## Required Secrets

Configure these secrets in GitHub repository settings:

### Docker Hub
- `DOCKER_USERNAME`: Docker Hub username
- `DOCKER_PASSWORD`: Docker Hub password or access token

### Codecov (optional)
- `CODECOV_TOKEN`: Codecov upload token

### Deployment (if using automated deployment)
- `STAGING_SSH_KEY`: SSH key for staging server
- `PRODUCTION_SSH_KEY`: SSH key for production server
- `STAGING_HOST`: Staging server hostname
- `PRODUCTION_HOST`: Production server hostname

## Running CI Locally

### Full CI Pipeline

```bash
# Run complete CI pipeline locally
make ci
```

This runs:
1. Code formatting (`cargo fmt`)
2. Linting (`cargo clippy`)
3. All tests (unit, integration, BDD)

### Individual Steps

```bash
# Format code
make fmt

# Run linter
make clippy

# Run tests
make test

# Run benchmarks
make bench

# Generate coverage
make coverage
```

## CI/CD Best Practices

### Branch Protection

Configure branch protection rules for `main`:

1. **Required status checks**:
   - Lint and Format
   - Test Suite
   - Security Scan

2. **Required reviews**: At least 1 approval

3. **Require branches to be up to date**: Yes

4. **Include administrators**: Yes

### Commit Messages

Follow conventional commits format:

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types**:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `style`: Formatting
- `refactor`: Code restructuring
- `perf`: Performance improvement
- `test`: Tests
- `chore`: Maintenance

**Examples**:
```
feat(memory): add zero-copy MMAP manager

Implements memory-mapped file management with:
- Region allocation
- Pool allocator
- Safety checks

Closes #42
```

### Pull Request Workflow

1. **Create feature branch**:
   ```bash
   git checkout -b feature/my-feature
   ```

2. **Make changes and commit**:
   ```bash
   git add .
   git commit -m "feat(scope): description"
   ```

3. **Push and create PR**:
   ```bash
   git push origin feature/my-feature
   ```

4. **CI runs automatically**:
   - Lint and format checks
   - Test suite
   - Security scans
   - Benchmark comparison

5. **Address feedback**:
   - Fix any CI failures
   - Respond to review comments
   - Push additional commits

6. **Merge when approved**:
   - All checks pass
   - At least 1 approval
   - No merge conflicts

### Release Process

1. **Update version**:
   ```bash
   # Update Cargo.toml version
   vim Cargo.toml
   
   # Update CHANGELOG.md
   vim CHANGELOG.md
   ```

2. **Commit version bump**:
   ```bash
   git add Cargo.toml CHANGELOG.md
   git commit -m "chore: bump version to 1.2.3"
   git push origin main
   ```

3. **Create and push tag**:
   ```bash
   git tag -a v1.2.3 -m "Release version 1.2.3"
   git push origin v1.2.3
   ```

4. **CI automatically**:
   - Runs full test suite
   - Builds production images
   - Tags images with version
   - Deploys to production
   - Creates GitHub release

## Monitoring CI/CD

### GitHub Actions Dashboard

View workflow runs at:
```
https://github.com/<owner>/<repo>/actions
```

### Status Badges

Add to README.md:

```markdown
![CI](https://github.com/<owner>/<repo>/workflows/CI/badge.svg)
![Deploy](https://github.com/<owner>/<repo>/workflows/Deploy/badge.svg)
[![codecov](https://codecov.io/gh/<owner>/<repo>/branch/main/graph/badge.svg)](https://codecov.io/gh/<owner>/<repo>)
```

### Notifications

Configure notifications in GitHub settings:
- Email on workflow failure
- Slack integration for deployments
- Discord webhooks for releases

## Troubleshooting

### CI Failures

**Lint failures**:
```bash
# Fix formatting locally
make fmt

# Fix clippy warnings
cargo clippy --fix --allow-dirty
```

**Test failures**:
```bash
# Run tests locally
make test

# Run specific test
cargo test test_name -- --nocapture

# Check logs
docker-compose logs jessy-core
```

**Coverage below threshold**:
```bash
# Generate coverage report
make coverage

# View report
open test-results/index.html

# Add missing tests
```

**Security vulnerabilities**:
```bash
# Check vulnerabilities
cargo audit

# Update dependencies
cargo update

# Check for breaking changes
cargo test
```

### Deployment Failures

**Image build failures**:
```bash
# Build locally
docker-compose build

# Check Dockerfile syntax
docker build -f docker/Dockerfile.rust .
```

**Deployment failures**:
```bash
# Check deployment logs in GitHub Actions
# Verify secrets are configured
# Test deployment manually
```

**Smoke test failures**:
```bash
# Check service health
curl https://staging.jessy.example.com/health

# Check logs
ssh staging-server "docker logs jessy-core"
```

## Performance Optimization

### Cache Strategy

**Cargo dependencies**:
- Cached by `Cargo.lock` hash
- Restored on every run
- Speeds up builds by 2-3x

**Docker layers**:
- BuildKit cache in GitHub Actions
- Layer caching for dependencies
- Speeds up image builds by 5-10x

**Test artifacts**:
- Coverage reports cached
- Benchmark baselines cached
- Reduces redundant work

### Parallel Execution

Jobs run in parallel when possible:
- Lint and Security run simultaneously
- Test and Build run after Lint
- Deployment waits for all checks

### Conditional Execution

- Build only on `main` branch
- Benchmarks only on PRs
- Deployment only on tags
- Reduces unnecessary work

## Security

### Secrets Management

- Never commit secrets to git
- Use GitHub Secrets for sensitive data
- Rotate secrets regularly
- Use least privilege access

### Vulnerability Scanning

- Trivy scans on every PR
- cargo audit on every PR
- Results uploaded to GitHub Security
- Fails on critical vulnerabilities

### Image Signing (future)

- Sign Docker images with Cosign
- Verify signatures before deployment
- Maintain chain of custody

## Related Documentation

- [Testing Infrastructure](TESTING_INFRASTRUCTURE.md)
- [Docker Setup](DOCKER_SETUP.md)
- [Development Workflow](DEVELOPMENT_PRINCIPLES.md)
- [Security Best Practices](SECURITY.md)

---

*"Automate everything. Trust nothing. Verify always. 🚀"*
