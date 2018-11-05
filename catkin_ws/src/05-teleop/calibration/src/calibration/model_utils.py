from abc import ABCMeta, abstractmethod
from collections import OrderedDict

class BaseModel(MetaClass = ABCMeta):
    def __init__(self):
        self.u = None
        self.s = None
        self.p = None
        self.c = None

    def defStates(self, model_states):
        self.s = OrderedDict()

        for state in model_states:
            self.s[state] = None

    def defInputs(self,model_inputs):
        self.u = OrderedDict()

        for input in model_inputs:
            self.u[input] = None

    def defParams(self, model_params):
        self.s = OrderedDict()

        for state in model_params:
            self.s[state] = None

    def defConstants(self, model_constants):
        self.c = OrderedDict()

        for constant in model_constants:
            self.s[constant] = None

    def setState(self, state_name, state_val):
        #expect an array or handle the scalar case. All operations should be an array: numpy??
        self.s[state_name] = state_val

    def setInput(self, input_name, input_val):
        #expect an array or handle the scalar case. All operations should be an array: numpy??
        self.u[input_name] = input_val

    def setParam(self, param_name, param_val):

        #expect an array or handle the scalar case. All operations should be an array: numpy??
        self.p[param_name] = param_val

    def setConstant(self, constant_name, constant_val):
        #expect an array or handle the scalar case. All operations should be an array: numpy??
        self.c[constant_name] = constant_val

    def getState(self, state_name):
        return self.s[state_name]

    def getInput(self, input_name):
        return self.u[input_name]

    def getParam(self, param_name):
        return  self.p[param_name]

    def getConstant(self, constant_name):
        return self.c[constant_name]

    @abstractmethod
    def defineDiffEq(self, s, u, p):
        pass


class GainTrimModel(BaseModel):
    model_name = "gain_trim_model"

    states = ["v_x", "omega"]
    inputs = ["cmd_right", "cmd_left"]
    params = ["gain", "trim"]
    constants = ["L"]

    def __init__(self):
        kinematic_model = BaseModel()

        self.model_name = self.model_name

        kinematic_model.defStates(model_states = self.states)
        kinematic_model.defInputs(model_inputs = self.inputs)
        kinematic_model.defParams(model_params = self.params)
        kinematic_model.defConstants(model_params = self.constants)

    def _defineDiffEq(self, s, u, p):
        # unpack model inputs
        cmd_right = self.getInput('cmd_right')
        cmd_left = self.getInput('cmd_left')

        # unpack model params
        gain = self.getParam('gain')
        trim = self.getParam('trim')

        # unpack model params
        cmd_right = self.getConstant('L')

        vx_pred = gain * (cmd_right + cmd_left) * 0.5 + (trim / 2) * (cmd_left - cmd_right) * 0.5
        omega_pred = (gain / L) * (cmd_right - cmd_left) * 0.5 - (trim / (2 * L)) * (cmd_right + cmd_left) * 0.5

        return [vx_pred, omega_pred]
