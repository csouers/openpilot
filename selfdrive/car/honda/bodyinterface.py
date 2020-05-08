from selfdrive.car.honda.bodystate import BodyState

class BodyInterface():
  def __init__(self, BP):
    self.BS = BodyState(BP)
    self.cp_bodyd_body = self.BS.get_bodyd_body_can_parser(BP)


  def update(self, can_strings):
    # ******************* do can recv *******************
    self.cp_bodyd_body.update_strings(can_strings)
    ret = self.BS.update(self.cp_bodyd_body)
    
    return ret
