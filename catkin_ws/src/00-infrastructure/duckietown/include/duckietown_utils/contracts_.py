from .detect_environment import on_duckiebot

from . import logger

__all__ = ['contract']



if on_duckiebot():
    using_fake_contracts = True
    logger.warning('Contracts are disabled becaused we are on Duckiebot.')
else:
    try:
        # use PyContracts if installed 
        from contracts import contract, all_disabled  # @UnusedImport
        if all_disabled:
            logger.warning('Using PyContracts, but it was disabled by the user.')
        else:
            logger.warning('Using PyContracts.')
        using_fake_contracts = False
    except ImportError:
        logger.warning('Contracts are disabled becaused PyContracts not found.')
        using_fake_contracts = True
    
if using_fake_contracts:
    def contract(**kwargs):  # @UnusedVariable
        def phi(f):
            return f
        return phi
