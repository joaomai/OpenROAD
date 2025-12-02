# Copyright (c) 2021, The Regents of the University of California
# All rights reserved.
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


# Put helper functions in a separate namespace so they are not visible
# too users in the global namespace.
namespace eval mcp {

sta::define_cmd_args "random_placement" {}
proc random_placement { args } {
  mcp::random_placement_cmd
}

sta::define_cmd_args "kernighan_lin_placement" {\
  [-compact] \
}
proc kernighan_lin_placement { args } {
  sta::parse_key_args "kernighan_lin_placement" args \
    keys {} flags {-compact}
  set compact [info exists flags(-compact)]
  mcp::kernighan_lin_placement_cmd $compact
}

}

