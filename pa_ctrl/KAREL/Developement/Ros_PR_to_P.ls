-- Software License Agreement (BSD License)
--
-- Copyright (c) 2012-2014, TU Delft Robotics Institute
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
--
--  * Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
--  * Redistributions in binary form must reproduce the above
--    copyright notice, this list of conditions and the following
--    disclaimer in the documentation and/or other materials provided
--    with the distribution.
--  * Neither the name of the TU Delft Robotics Institute,nor the University Of Sherbrooke nor the names 
--    of its contributors may be used to endorse or promote products 
--    derived from this software without specific prior written 
--    permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
-- "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
-- LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
-- FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
-- COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
-- INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
-- BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
-- LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
-- CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
-- LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
-- ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.

PROGRAM ros_pr_to_p

%ALPHABETIZE
%COMMENT = 'r23'
%NOBUSYLAMP
%NOLOCKGROUP
%NOPAUSE = COMMAND + TPENABLE + ERROR

VAR
    xyz         : XYZWPREXT
    open_id     : INTEGER

CONST

--------------------------------------------------------------------------------
-- 
-- Main program
-- 
--------------------------------------------------------------------------------
BEGIN

    -- Open the to_prog with the Read/Write access
    OPEN_TPE (prog_name,TPE_RWACC,TPE_NOREJ,open_id,stat_)
    

    xyz = GET_POS_REG(1,stat_)
    SET_POS_TPE(10,10)

    CLOSE_TPE(open_id,stat_)

END ros_pr_to_p