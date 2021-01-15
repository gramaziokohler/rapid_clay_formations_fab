MODULE A052_DataGlobal
    !***********************************************************************************
    !
    ! ETH Z�rich / Robotic Fabrication Lab
    ! HIB C 13 / Stefano-Franscini-Platz 1
    ! CH-8093 Z�rich
    !
    !***********************************************************************************
    !
    ! PROJECT     :  A052 Robot Beefer
    !
    ! FUNCTION    :  Modul includs all global data  
    !
    ! AUTHOR      :  Philippe Fleischmann
    !
    ! EMAIL       :  fleischmann@arch.ethz.ch
    !
    ! HISTORY     :  2019.03.28 Draft
    !
    ! Copyright   :  ETH Z�rich (CH) 2018
    !                - Philippe Fleischmann
    !                - Michael Lyrenmann
    !                - Matthias Kohler 
    !
    ! License     :  You agree that the software source code and documentation
    !                provided by the copyright holder is confidential, 
    !                and you shall take all reasonable precautions to protect
    !                the source code and documentation, and preserve its confidential,
    !                proprietary and trade secret status in perpetuity. 
    ! 
    !                This license is strictly limited to INTERNAL use within one site.
    !
    !***********************************************************************************

    !************************************************
    ! Declaration :     bool
    !************************************************
    !
    PERS bool b_A052_StartGrill;
    PERS bool b_A052_GrillSide1;
    PERS bool b_A052_GrillSide2;

    !************************************************
    ! Declaration :     num
    !************************************************
    !
    PERS num n_A052_GrillTimeSide1;
    PERS num n_A052_GrillTimeSide2;
    PERS num n_A052_CurrentBeefTime;
    PERS num n_A052_MaxBeefTime;

    !************************************************
    ! Declaration :     string
    !************************************************
    !

ENDMODULE