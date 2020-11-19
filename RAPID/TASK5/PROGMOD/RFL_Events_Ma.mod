MODULE RFL_Events_Ma
    !***********************************************************************************
    !
    ! ETH Z�rich / Robotic Fabrication Lab
    ! HIB C 13 / Stefano-Franscini-Platz 1
    ! CH-8093 Z�rich
    !
    !***********************************************************************************
    !
    ! PROJECT     :  A011_RFL
    !
    ! FUNCTION    :  Event Routines
    !
    ! AUTHOR      :  Philippe Fleischmann
    !
    ! EMAIL       :  fleischmann@arch.ethz.ch
    !
    ! HISTORY     :  2018.05.24 Draft
    !
    ! Copyright   :  ETH Z�rich (CH) 2018
    !
    !***********************************************************************************

    !************************************************
    ! Function    :     Event Power On
    ! Programmer  :     Philippe Fleischmann
    ! Date        :     2018.05.24
    !***************** ETH Z�rich *******************
    !
    PROC r_RFL_EvePowOn()
        !
        ! Set the resetsignal to show the Master menue
        SetDo doMaFP3,1;
        RETURN ;
    ERROR
        ! Placeholder for Error Code...
    ENDPROC
ENDMODULE