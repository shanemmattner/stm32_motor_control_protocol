# Publishing to PyPI

This guide explains how to publish `stm32-motor-control-protocol` to the Python Package Index (PyPI).

## Prerequisites

1. **PyPI Account**: Create account at https://pypi.org/account/register/
2. **GitHub**: Repository with write access
3. **uv**: Modern Python package manager

## Setup (One-Time)

### Step 1: Create PyPI API Token

1. Go to https://pypi.org/account/token/
2. Click "Add API token"
3. Name: `stm32-motor-control-protocol`
4. Scope: Entire account
5. Copy the token (starts with `pypi-`)

### Step 2: Add GitHub Secret

1. Go to repository Settings → Secrets and variables → Actions
2. Click "New repository secret"
3. Name: `PYPI_API_TOKEN`
4. Value: Paste the PyPI token
5. Click "Add secret"

### Step 3: Verify Configuration

Check these files are in place:
- ✅ `pyproject.toml` - Project metadata
- ✅ `LICENSE` - MIT license
- ✅ `README.md` - Project description
- ✅ `.github/workflows/publish.yml` - Publishing workflow
- ✅ `CHANGELOG.md` - Release notes

## Publishing a Release

### Automatic Publishing (Recommended)

1. **Update version** in `pyproject.toml`:
   ```toml
   [project]
   version = "0.2.0"
   ```

2. **Update CHANGELOG.md**:
   ```markdown
   ## [0.2.0] - 2025-11-10

   ### Added
   - New feature description

   ### Fixed
   - Bug fix description
   ```

3. **Commit and push**:
   ```bash
   git add pyproject.toml CHANGELOG.md
   git commit -m "chore: bump version to 0.2.0"
   git push origin main
   ```

4. **Create git tag**:
   ```bash
   git tag -a v0.2.0 -m "Release version 0.2.0"
   git push origin v0.2.0
   ```

5. **Automatic actions**:
   - GitHub Actions builds the package
   - Publishes to PyPI
   - Creates GitHub Release with assets

### Manual Publishing (Fallback)

If automated publishing fails:

```bash
# Install build tools
uv add --dev build twine

# Build distribution
uv run python -m build

# Check distribution
uv run twine check dist/*

# Upload to PyPI (using API token)
uv run twine upload dist/* -u __token__ -p pypi-YOUR_TOKEN_HERE
```

## Version Management

### Semantic Versioning

Follow [Semantic Versioning](https://semver.org/):

- **MAJOR** (0.2.0 → 1.0.0): Breaking changes
- **MINOR** (0.1.0 → 0.2.0): New features (backward compatible)
- **PATCH** (0.1.0 → 0.1.1): Bug fixes (backward compatible)

Examples:
- `0.1.0` - Initial release
- `0.2.0` - New motor control feature
- `0.2.1` - Bug fix in communication
- `1.0.0` - Stable API, registry service complete

### Beta Releases

For pre-release versions:
```
0.2.0a1    # Alpha release
0.2.0b1    # Beta release
0.2.0rc1   # Release candidate
```

Tag: `v0.2.0a1`, `v0.2.0b1`, `v0.2.0rc1`

These will be marked as pre-releases on PyPI.

## Files Included in Distribution

The `MANIFEST.in` is handled automatically. Includes:

✅ Python source files (`st_mcp/`)
✅ Tests (`tests/`)
✅ Documentation files (README.md, LICENSE, etc.)
✅ Protocol documentation (PROTOCOL_DOCUMENTATION.md)
✅ Serial traffic logs (reference data)

Excludes:
- `.git/`
- `__pycache__/`
- `.tox/`, `.pytest_cache/`
- Build artifacts

## Checking Before Publishing

### Local Build Test

```bash
# Build package
uv run python -m build

# Check contents
unzip -l dist/stm32_motor_control_protocol-0.2.0-py3-none-any.whl

# Verify metadata
uv run twine check dist/*
```

### Test PyPI (Recommended)

Publish to test.pypi.org first:

```bash
# Get token from https://test.pypi.org/account/token/
# Add as PYPI_TEST_API_TOKEN secret

# In publish.yml, test first:
uv run twine upload --repository testpypi dist/* \
  -u __token__ -p ${{ secrets.PYPI_TEST_API_TOKEN }}
```

Then install to verify:
```bash
pip install -i https://test.pypi.org/simple/ \
  stm32-motor-control-protocol==0.2.0
```

## Troubleshooting

### "Package already exists"

PyPI doesn't allow overwriting releases. If needed:

1. **Test PyPI first**: Publish to test.pypi.org
2. **Pre-release versions**: Use `0.2.0a1`, `0.2.0b1`, etc.
3. **Yanked releases**: Mark old version as yanked on PyPI website
4. **New version**: Always increment to new version

### "Invalid API token"

```
Error: Invalid API token
```

Solutions:
1. Verify token in GitHub Secrets (Settings → Secrets)
2. Check token hasn't expired (regenerate if needed)
3. Ensure correct token (not old one)
4. Test locally: `uv run twine --version`

### "Credentials not provided"

Check `.pypirc` doesn't override tokens:
```bash
cat ~/.pypirc  # Should NOT exist or be empty
```

### Build fails

```bash
# Clean and rebuild
rm -rf dist/ build/ *.egg-info
uv run python -m build
uv run twine check dist/*
```

## After Publishing

### Verify on PyPI

1. Visit https://pypi.org/project/stm32-motor-control-protocol/
2. Check version is listed
3. Verify metadata is correct
4. Test installation:

```bash
pip install stm32-motor-control-protocol
python -c "from st_mcp import __version__; print(__version__)"
```

### Announce Release

- Post on project discussions/announcements
- Update README with installation instructions
- Tag contributors in GitHub Release

## Security Best Practices

### API Token Security

✅ Use repository secrets (not in code)
✅ Use fine-grained tokens (per package)
✅ Rotate tokens regularly
✅ Use separate tokens for test/prod PyPI

### Build Verification

✅ Always test locally before publishing
✅ Use GitHub Actions for automated builds
✅ Never commit credentials
✅ Review dependencies before release

## Advanced Topics

### Artifact Signing

For additional security, sign packages:

```bash
# Generate GPG key (one-time)
gpg --gen-key

# Sign distributions
gpg --sign --detach-sign dist/*.whl

# Upload with signature
twine upload dist/* *.asc
```

### Publish to Multiple Registries

Create separate workflows for:
- Production PyPI: `v*` tags
- Test PyPI: `v*-test` tags
- Private registry: Custom condition

## Resources

- [PyPI Help](https://pypi.org/help/)
- [Python Packaging](https://packaging.python.org/)
- [Semantic Versioning](https://semver.org/)
- [GitHub Actions](https://docs.github.com/actions)
- [uv Documentation](https://github.com/astral-sh/uv)

## Publishing Checklist

Before tagging a release:

- [ ] Update `pyproject.toml` version
- [ ] Update `CHANGELOG.md`
- [ ] Run `make all` (passes all checks)
- [ ] Test locally: `uv run python -m build`
- [ ] Commit: `git commit -m "chore: bump version"`
- [ ] Push: `git push origin main`
- [ ] Tag: `git tag -a v0.2.0 -m "Release 0.2.0"`
- [ ] Push tag: `git push origin v0.2.0`
- [ ] Verify on PyPI: https://pypi.org/project/stm32-motor-control-protocol/
- [ ] Verify installation: `pip install stm32-motor-control-protocol`

That's it! The rest is automated.
