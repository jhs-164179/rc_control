"""Optional CPU acceleration with an unchanged NumPy fallback.

Select a backend before starting analysis workers. Only dependency/compiler
failures fall back in auto mode; measurement errors are never swallowed.
"""
from __future__ import annotations
import importlib
import os
import threading
import warnings
import numpy as np

_MODES = ('auto', 'numpy', 'numba')
_mode = os.environ.get('CRATER_BACKEND', 'auto').strip().lower()
if _mode not in _MODES:
    raise ValueError('CRATER_BACKEND must be auto, numpy or numba')
_lock = threading.RLock()
_module = None
_attempted = False
_failure = None
_compiler_errors = ()
_KERNELS = ('grouped_medians', 'cell_statistics', 'profile_cells', 'component_count')


def _unavailable(error):
    global _module, _failure
    _module = None
    _failure = f'{type(error).__name__}: {error}'
    if _mode == 'numba':
        raise RuntimeError('Numba acceleration unavailable: '+_failure) from error
    warnings.warn('Numba acceleration unavailable; using NumPy. '+_failure,
                  RuntimeWarning, stacklevel=3)


def _get_module():
    global _module, _attempted, _compiler_errors
    if _mode == 'numpy':
        return None
    if _attempted:
        if _module is None and _mode == 'numba':
            raise RuntimeError('Numba acceleration unavailable: '+str(_failure))
        return _module
    with _lock:
        if _attempted:
            return _get_module()
        try:
            numba = importlib.import_module('numba')
            if numba.config.DISABLE_JIT:
                raise RuntimeError('NUMBA_DISABLE_JIT is enabled')
            from numba.core.errors import NumbaError
            _compiler_errors = (NumbaError, OSError)
            _module = importlib.import_module('._numba_kernels', __package__)
        except (ImportError, OSError, RuntimeError) as error:
            _unavailable(error)
        finally:
            _attempted = True
        return _module


def accelerate(name, *args):
    """Return a kernel result, or None to execute the original NumPy branch."""
    if name not in _KERNELS:
        raise ValueError('unknown acceleration kernel: '+name)
    module = _get_module()
    if module is None:
        return None
    try:
        return getattr(module, name)(*args)
    except _compiler_errors as error:
        with _lock:
            _unavailable(error)
        return None


def configure_backend(mode='auto'):
    """Configure before starting workers: auto (default), numpy, or numba.

    auto uses NumPy when dependencies/compiler support are unavailable;
    numba requires working JIT support and raises on infrastructure failures.
    """
    global _mode, _attempted, _failure
    mode = str(mode).strip().lower()
    if mode not in _MODES:
        raise ValueError('backend must be auto, numpy or numba')
    with _lock:
        _mode = mode
        if _module is None:
            _attempted = False
            _failure = None
    return backend_status()


def backend_status():
    """Report dependency availability and which kernels have been compiled."""
    module = _get_module()
    return {
        'requested': _mode,
        'active': 'numba' if module is not None else 'numpy',
        'fallback_reason': _failure,
        'compiled_signatures': {
            name: len(getattr(module, name).nopython_signatures) if module is not None else 0
            for name in _KERNELS},
        'fastmath': False,
        'parallel': False,
        'numpy_version': np.__version__,
        'numba_version': importlib.import_module('numba').__version__ if module is not None else None,
    }


def warmup():
    """Compile common float64/integer layouts without processing sensor data.

    Call once in the actual analysis process before starting workers. A
    different input dtype/layout may still require a new specialization.
    """
    if _get_module() is None:
        return backend_status()
    values = np.arange(18, dtype=np.float64).reshape(6, 3)
    residual = values[:, 0].copy()
    for dtype in {np.dtype(int), np.dtype(np.intp), np.dtype(np.int64)}:
        breaks = np.array([0, 3, 6], dtype=dtype)
        accelerate('grouped_medians', values, breaks)
        accelerate('cell_statistics', values, residual, breaks, 3)
        centers = (np.arange(12)+.5)*.1
        rho = np.array([.12, .14, .16, .22, .24, .26])
        sectors = np.zeros(6, dtype=dtype)
        accelerate('profile_cells', rho, sectors, residual, residual, centers, .1, 12, 3, True)
        edges = np.array([[0, 1], [1, 2], [0, 2]], dtype=dtype)
        accelerate('component_count', 3, edges, np.arange(3, dtype=dtype))
    return backend_status()
