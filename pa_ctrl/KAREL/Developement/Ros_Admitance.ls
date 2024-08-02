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


PROGRAM ros_admittance
--------------------------------------------------------------------------------
-- 
-- ROS-Industrial force admittance.
-- 
-- 
-- Assumptions:
--   - There is only 1 motion group
-- 
-- 
-- Configuration defaults:
--   - TAG '53' used for USM
--   - force TCP port ROS-I default (11004)
-- 
-- 
-- author: Philippe D'Amours (University of Sherbrooke); G.A. vd. Hoorn (TU Delft Robotics Institute)
-- 
--------------------------------------------------------------------------------
%ALPHABETIZE
%COMMENT = 'r23'
%NOBUSYLAMP
%NOLOCKGROUP
%NOPAUSE = COMMAND + TPENABLE + ERROR




--------------------------------------------------------------------------------
-- 
-- remote types & constants
-- 
--------------------------------------------------------------------------------
%INCLUDE include\libssock_t
%INCLUDE include\libind_pkt_t
%INCLUDE include\libind_rs_t




--------------------------------------------------------------------------------
-- 
-- local types & constants
-- 
--------------------------------------------------------------------------------
TYPE
	rforce_cfg_t = STRUCTURE
		checked      : BOOLEAN  -- user flagged: config checked
		loop_hz      : INTEGER  -- main loop update rate (in Hz)
		sloop_div    : INTEGER  -- robot_status main loop divider
		s_tcp_nr     : INTEGER  -- TCP port to listen on
		s_tag_nr     : INTEGER  -- server TAG number to use
		um_clear     : BOOLEAN  -- clear user menu on start
	ENDSTRUCTURE


VAR
	cfg_         IN SHADOW : rforce_cfg_t -- configuration

	--jnt_pkt_out_           : indp_kt_t    -- ROS-Industrial joint state packet
	--rs_pkt_out_            : ind_rs_t     -- ROS-Industrial robot status packet
	sock_fd_               : FILE         -- file descriptor associated with srvr socket
	stat_                  : INTEGER      -- status variable
	stat_count_            : INTEGER      -- status loop counter
	--cur_j_pos_             : JOINTPOS     -- current position of robot joints
	sleep_time_            : INTEGER
	shutdwn_req_           : BOOLEAN      -- program abort requested status
	cur_f_                 : ARRAY[6] OF REAL -- current force at effector -- by PDA
	i__                    : INTEGER      -- for iteration
	array_sz_f__           : INTEGER      -- size of array of force
	entry                  : INTEGER      -- for GET_VAR

	-- admittance varaible 
	xc_a				   : REAL		  -- Acceleration de la position compliant
	xc_v				   : REAL		  -- Vitesse      de la position compliant
	xc_p				   : REAL		  -- Position     de la position compliant
	-- Position désiré initial est 0, on calcul la différence qui sera ajouté à la position 
	xd_a				   : REAL		  -- Acceleration de la position désirée initial
	xd_v				   : REAL		  -- Vitesse      de la position désirée initial
	xd_p				   : REAL		  -- Delta de Position     de la position désiré
	-- Position xyzwpr compliant
	COMP_POS			   : XYZWPR

	-- Constante pour la boucle d'admittance
	MASSE				   : REAL   -- Masse de la masse virtuel
	KP					   : REAL   -- Constante ressort du systeme virtuel
	KD					   : REAL	-- Constante d'amortissement du systeme virtuel
	DES_F				   : REAL   -- Force desire pour 

	r_flg				   : BOOLEAN -- Flag of REAL or INT pour la fonction GET_REG
	i_val				   : INTEGER -- Valeur int de GET_REG si r_flg


CONST
	-- 
	LOG_PFIX     = 'RFOR '

	COND_AH      =    1  -- ABORT handler id

	CFG_OK       =    0  -- config ok
	CFG_NOTDONE  =   -1  -- configuration not checked: user action required

	HOST_CTAG_ER = 67144 -- HOST-144 Comm Tag error
	FILE_ILL_PRM =  2032 -- FILE-032 Illegal parameter
	SEV_ABORT    =    2  -- ABORT severity



	-- Configuration defaults
	FORCE_TAG    =    5  -- Server tag
	FORCE_TCP_P  = 11004 -- Socket
	LOOP_HZ      =   42  -- Main loop freq (Hz).
	                     -- Note: we configure 42 Hz here, as due to ITP-bound
	                     --       scheduling, anything below this (or actually
	                     --       below 41.67 Hz) will result in an effective
	                     --       frequency of 27 (for 12ms ITPs) or 31 Hz
	                     --       (for 8ms ITPs). See also issue 203 on GH.
	STAT_LP_CNT  =   10  -- Main loop divider for robot_status
	--MASSE		 =	 1   -- Masse de la masse virtuel
	--KP			 = 	 1   -- Constante ressort du systeme virtuel
	--KD			 =	 1	 -- Constante d'amortissement du systeme virtuel
	--DES_F		 = 	 2	 -- Force desiré 




--------------------------------------------------------------------------------
-- 
-- remote routine prototypes
-- 
--------------------------------------------------------------------------------
%INCLUDE include\libssock_h
%INCLUDE include\libind_pkt_h
%INCLUDE include\libind_hdr_h
%INCLUDE include\libind_log_h
%INCLUDE include\libind_mth_h
%INCLUDE include\libind_rs_h




--------------------------------------------------------------------------------
-- 
-- local routine prototypes
-- 
--------------------------------------------------------------------------------
ROUTINE check_cfg_(cfg : rforce_cfg_t) : INTEGER FROM ros_admittance
ROUTINE install_ah_ FROM ros_admittance
ROUTINE remove_ah_ FROM ros_admittance




--------------------------------------------------------------------------------
-- 
-- Main program
-- 
--------------------------------------------------------------------------------
BEGIN
	-- check config
	stat_ = check_cfg_(cfg_)
	IF (stat_ <> CFG_OK) THEN
		log_error_a(LOG_PFIX + 'cfg error:', stat_)
		log_error(LOG_PFIX + 'check cfg')
		-- errors with config always force user to log window
		log_force
		POST_ERR(FILE_ILL_PRM, '', 0, SEV_ABORT)
		RETURN
	ENDIF

	


	-- 
	stat_        = 0
	stat_count_  = 0
	sleep_time_  = ROUND(1000.0 / cfg_.loop_hz)
	shutdwn_req_ = FALSE
	i__          = 0
	array_sz_f__ = 6

	--Get les variables pour la loop d'admittance
	GET_REG(1,r_flg,i_val,MASSE,stat_)
	GET_REG(2,r_flg,i_val,KP,stat_)
	GET_REG(3,r_flg,i_val,KD,stat_)
	GET_REG(4,r_flg,i_val,DES_F,stat_)



	-- initialise ABORT handler
	install_ah_


	-- enable log output
	IF (cfg_.um_clear) THEN log_clear; ENDIF


	-- init packet - No more packet
	--stat_ = ipkt_ctor(jnt_pkt_out_)
	-- we reply with JOINT_POSITION packets
	--USING jnt_pkt_out_ DO
	--	length_    = (RI_SZ_HDR + RI_SZB_JPOS)
	--	msg_type_  = RI_MT_JOINTP
	--	comm_type_ = RI_CT_TOPIC
		-- rest is set by ctor
	--ENDUSING


	---- init server socket
	--stat_ = ssock_ctor(sock_, cfg_.s_tcp_nr,  cfg_.s_tag_nr)
	--IF (stat_ <> 0) THEN
	--	IF (stat_ = TAG_CONF_ERR) THEN
	--		log_error_a(LOG_PFIX + 'cfg err, TAG idx:', cfg_.s_tag_nr)
	--	ELSE
	--		log_error_a(LOG_PFIX + 'ssock_ctor err:', stat_)
	--	ENDIF
	--	-- nothing we can do, abort
	--	POST_ERR(HOST_CTAG_ER, '', 0, SEV_ABORT)
	--ENDIF


	-- make sure socket is closed
	-- don t care about result
	--stat_ = ssock_dconnf(sock_)

	-- 
	WHILE (NOT shutdwn_req_) DO

		-- inform user
		log_info(LOG_PFIX + 'Starting admittance loop')

		-- set socket in 'binary mode' (unformatted)
		-- KAREL Reference Manual, MARRC75KR07091E Rev C, sec 7.3.1
		--SET_FILE_ATR(sock_fd_, ATR_UF)

		-- wait for connection
		--stat_ = ssock_accpt2(sock_, sock_fd_)
		--IF (stat_ <> 0) THEN
		--	log_error_a(LOG_PFIX + 'sock_accept err:', stat_)
		--	-- can't continue
		--	GOTO exit_discon
		--ENDIF

		-- inform user
		

		-- got client, start force relay loop
		WHILE (NOT shutdwn_req_) DO

			-- OLD exemple
			-- get current joint angles 
			-- cur_j_pos_ = CURJPOS(0, 0)

			-- get current force at effector 
			GET_VAR(entry, '*SYSTEM*', '$DCSS_CLLB[1].$EXT_FORCE', cur_f_, stat_)
			log_info(LOG_PFIX +'FORCE:')
			log_info(cur_f_)

			-- Exemple d'admittance calculation
			--self.xc_a = self.xd_a + 1.0/self.m*(self.f_mes - self.f_des - self.kd*(self.xc_v - self.xd_v) - self.kp*(self.xc_p - self.xd_p))
        	--self.xc_v = self.xc_v + self.xc_a*self.ts
        	--self.xc_p = self.xc_p + self.xc_v*self.ts

			-- Boucle d'admittance
			xc_a = xd_a + 1.0/MASSE*(cur_f_[3] - DES_F - KD*(xc_v - xd_v) - KP*(xc_p - xd_p))
			xc_v = xc_v + xc_a*sleep_time_
			xc_p = xc_p + xc_v*sleep_time_

			-- Load the position in Position Register
			
			COMP_POS = GET_POS_REG(1, stat_)
			
			
			COMP_POS.y = COMP_POS.y + xc_p

			SET_POS_REG(3,COMP_POS,stat_)

			DELAY sleep_time_

		-- inner WHILE TRUE DO
		ENDWHILE

		-- exit with forced disconnect
		--exit_discon::
		--	stat_ = ssock_dconn2(sock_, sock_fd_)

		-- exit inner loop
		exit_inner::
			-- nothing

	-- outer WHILE TRUE DO
	ENDWHILE

exit_on_err::
	-- nothing

	-- make sure socket is closed (don t care about result)
	--stat_ = ssock_dconnf(sock_)

	-- disable the ABORT handler
	remove_ah_

	-- done
	log_info(LOG_PFIX + 'Exit')

END ros_admittance




ROUTINE check_cfg_
VAR
	a__ : BOOLEAN
BEGIN
	a__ = FALSE

	-- set defaults for any uninitialised entries
	IF (UNINIT(cfg.loop_hz  )) THEN cfg.loop_hz   = LOOP_HZ    ; a__ = TRUE; ENDIF
	IF (UNINIT(cfg.sloop_div)) THEN cfg.sloop_div = STAT_LP_CNT; a__ = TRUE; ENDIF
	IF (UNINIT(cfg.s_tcp_nr )) THEN cfg.s_tcp_nr  = FORCE_TCP_P; a__ = TRUE; ENDIF
	IF (UNINIT(cfg.s_tag_nr )) THEN cfg.s_tag_nr  = FORCE_TAG  ; a__ = TRUE; ENDIF
	IF (UNINIT(cfg.um_clear )) THEN cfg.um_clear  = TRUE       ; a__ = TRUE; ENDIF
	IF (UNINIT(cfg.checked  )) THEN cfg.checked   = FALSE      ; a__ = TRUE; ENDIF

	-- make sure user has checked configuration
	IF ((NOT cfg.checked) OR a__) THEN RETURN (CFG_NOTDONE); ENDIF

	-- all ok
	RETURN (CFG_OK)
END check_cfg_




--------------------------------------------------------------------------------
-- 
-- Application handler for ABORT signals.
-- 
--------------------------------------------------------------------------------
ROUTINE ab_hndlr_
BEGIN
	shutdwn_req_ = TRUE
	CANCEL FILE sock_fd_
END ab_hndlr_




--------------------------------------------------------------------------------
-- 
-- Installs a condition handler that catches ABORT signals to allow the
-- application to 'gracefully' exit.
-- 
--------------------------------------------------------------------------------
ROUTINE install_ah_
BEGIN
	CONDITION[COND_AH]: WITH $SCAN_TIME = 256
		WHEN ABORT DO
			NOABORT
			ab_hndlr_
	ENDCONDITION
	ENABLE CONDITION[COND_AH]
END install_ah_




--------------------------------------------------------------------------------
-- 
-- Deregisters the ABORT condition handler.
-- 
--------------------------------------------------------------------------------
ROUTINE remove_ah_
BEGIN
	PURGE CONDITION[COND_AH]
END remove_ah_
