(cl:defpackage vrep_common-srv
  (:use )
  (:export
   "SIMROSADDSTATUSBARMESSAGE"
   "<SIMROSADDSTATUSBARMESSAGE-REQUEST>"
   "SIMROSADDSTATUSBARMESSAGE-REQUEST"
   "<SIMROSADDSTATUSBARMESSAGE-RESPONSE>"
   "SIMROSADDSTATUSBARMESSAGE-RESPONSE"
   "SIMROSAPPENDSTRINGSIGNAL"
   "<SIMROSAPPENDSTRINGSIGNAL-REQUEST>"
   "SIMROSAPPENDSTRINGSIGNAL-REQUEST"
   "<SIMROSAPPENDSTRINGSIGNAL-RESPONSE>"
   "SIMROSAPPENDSTRINGSIGNAL-RESPONSE"
   "SIMROSAUXILIARYCONSOLECLOSE"
   "<SIMROSAUXILIARYCONSOLECLOSE-REQUEST>"
   "SIMROSAUXILIARYCONSOLECLOSE-REQUEST"
   "<SIMROSAUXILIARYCONSOLECLOSE-RESPONSE>"
   "SIMROSAUXILIARYCONSOLECLOSE-RESPONSE"
   "SIMROSAUXILIARYCONSOLEOPEN"
   "<SIMROSAUXILIARYCONSOLEOPEN-REQUEST>"
   "SIMROSAUXILIARYCONSOLEOPEN-REQUEST"
   "<SIMROSAUXILIARYCONSOLEOPEN-RESPONSE>"
   "SIMROSAUXILIARYCONSOLEOPEN-RESPONSE"
   "SIMROSAUXILIARYCONSOLEPRINT"
   "<SIMROSAUXILIARYCONSOLEPRINT-REQUEST>"
   "SIMROSAUXILIARYCONSOLEPRINT-REQUEST"
   "<SIMROSAUXILIARYCONSOLEPRINT-RESPONSE>"
   "SIMROSAUXILIARYCONSOLEPRINT-RESPONSE"
   "SIMROSAUXILIARYCONSOLESHOW"
   "<SIMROSAUXILIARYCONSOLESHOW-REQUEST>"
   "SIMROSAUXILIARYCONSOLESHOW-REQUEST"
   "<SIMROSAUXILIARYCONSOLESHOW-RESPONSE>"
   "SIMROSAUXILIARYCONSOLESHOW-RESPONSE"
   "SIMROSBREAKFORCESENSOR"
   "<SIMROSBREAKFORCESENSOR-REQUEST>"
   "SIMROSBREAKFORCESENSOR-REQUEST"
   "<SIMROSBREAKFORCESENSOR-RESPONSE>"
   "SIMROSBREAKFORCESENSOR-RESPONSE"
   "SIMROSCALLSCRIPTFUNCTION"
   "<SIMROSCALLSCRIPTFUNCTION-REQUEST>"
   "SIMROSCALLSCRIPTFUNCTION-REQUEST"
   "<SIMROSCALLSCRIPTFUNCTION-RESPONSE>"
   "SIMROSCALLSCRIPTFUNCTION-RESPONSE"
   "SIMROSCLEARFLOATSIGNAL"
   "<SIMROSCLEARFLOATSIGNAL-REQUEST>"
   "SIMROSCLEARFLOATSIGNAL-REQUEST"
   "<SIMROSCLEARFLOATSIGNAL-RESPONSE>"
   "SIMROSCLEARFLOATSIGNAL-RESPONSE"
   "SIMROSCLEARINTEGERSIGNAL"
   "<SIMROSCLEARINTEGERSIGNAL-REQUEST>"
   "SIMROSCLEARINTEGERSIGNAL-REQUEST"
   "<SIMROSCLEARINTEGERSIGNAL-RESPONSE>"
   "SIMROSCLEARINTEGERSIGNAL-RESPONSE"
   "SIMROSCLEARSTRINGSIGNAL"
   "<SIMROSCLEARSTRINGSIGNAL-REQUEST>"
   "SIMROSCLEARSTRINGSIGNAL-REQUEST"
   "<SIMROSCLEARSTRINGSIGNAL-RESPONSE>"
   "SIMROSCLEARSTRINGSIGNAL-RESPONSE"
   "SIMROSCLOSESCENE"
   "<SIMROSCLOSESCENE-REQUEST>"
   "SIMROSCLOSESCENE-REQUEST"
   "<SIMROSCLOSESCENE-RESPONSE>"
   "SIMROSCLOSESCENE-RESPONSE"
   "SIMROSCOPYPASTEOBJECTS"
   "<SIMROSCOPYPASTEOBJECTS-REQUEST>"
   "SIMROSCOPYPASTEOBJECTS-REQUEST"
   "<SIMROSCOPYPASTEOBJECTS-RESPONSE>"
   "SIMROSCOPYPASTEOBJECTS-RESPONSE"
   "SIMROSCREATEDUMMY"
   "<SIMROSCREATEDUMMY-REQUEST>"
   "SIMROSCREATEDUMMY-REQUEST"
   "<SIMROSCREATEDUMMY-RESPONSE>"
   "SIMROSCREATEDUMMY-RESPONSE"
   "SIMROSDISABLEPUBLISHER"
   "<SIMROSDISABLEPUBLISHER-REQUEST>"
   "SIMROSDISABLEPUBLISHER-REQUEST"
   "<SIMROSDISABLEPUBLISHER-RESPONSE>"
   "SIMROSDISABLEPUBLISHER-RESPONSE"
   "SIMROSDISABLESUBSCRIBER"
   "<SIMROSDISABLESUBSCRIBER-REQUEST>"
   "SIMROSDISABLESUBSCRIBER-REQUEST"
   "<SIMROSDISABLESUBSCRIBER-RESPONSE>"
   "SIMROSDISABLESUBSCRIBER-RESPONSE"
   "SIMROSDISPLAYDIALOG"
   "<SIMROSDISPLAYDIALOG-REQUEST>"
   "SIMROSDISPLAYDIALOG-REQUEST"
   "<SIMROSDISPLAYDIALOG-RESPONSE>"
   "SIMROSDISPLAYDIALOG-RESPONSE"
   "SIMROSENABLEPUBLISHER"
   "<SIMROSENABLEPUBLISHER-REQUEST>"
   "SIMROSENABLEPUBLISHER-REQUEST"
   "<SIMROSENABLEPUBLISHER-RESPONSE>"
   "SIMROSENABLEPUBLISHER-RESPONSE"
   "SIMROSENABLESUBSCRIBER"
   "<SIMROSENABLESUBSCRIBER-REQUEST>"
   "SIMROSENABLESUBSCRIBER-REQUEST"
   "<SIMROSENABLESUBSCRIBER-RESPONSE>"
   "SIMROSENABLESUBSCRIBER-RESPONSE"
   "SIMROSENDDIALOG"
   "<SIMROSENDDIALOG-REQUEST>"
   "SIMROSENDDIALOG-REQUEST"
   "<SIMROSENDDIALOG-RESPONSE>"
   "SIMROSENDDIALOG-RESPONSE"
   "SIMROSERASEFILE"
   "<SIMROSERASEFILE-REQUEST>"
   "SIMROSERASEFILE-REQUEST"
   "<SIMROSERASEFILE-RESPONSE>"
   "SIMROSERASEFILE-RESPONSE"
   "SIMROSGETANDCLEARSTRINGSIGNAL"
   "<SIMROSGETANDCLEARSTRINGSIGNAL-REQUEST>"
   "SIMROSGETANDCLEARSTRINGSIGNAL-REQUEST"
   "<SIMROSGETANDCLEARSTRINGSIGNAL-RESPONSE>"
   "SIMROSGETANDCLEARSTRINGSIGNAL-RESPONSE"
   "SIMROSGETARRAYPARAMETER"
   "<SIMROSGETARRAYPARAMETER-REQUEST>"
   "SIMROSGETARRAYPARAMETER-REQUEST"
   "<SIMROSGETARRAYPARAMETER-RESPONSE>"
   "SIMROSGETARRAYPARAMETER-RESPONSE"
   "SIMROSGETBOOLEANPARAMETER"
   "<SIMROSGETBOOLEANPARAMETER-REQUEST>"
   "SIMROSGETBOOLEANPARAMETER-REQUEST"
   "<SIMROSGETBOOLEANPARAMETER-RESPONSE>"
   "SIMROSGETBOOLEANPARAMETER-RESPONSE"
   "SIMROSGETCOLLECTIONHANDLE"
   "<SIMROSGETCOLLECTIONHANDLE-REQUEST>"
   "SIMROSGETCOLLECTIONHANDLE-REQUEST"
   "<SIMROSGETCOLLECTIONHANDLE-RESPONSE>"
   "SIMROSGETCOLLECTIONHANDLE-RESPONSE"
   "SIMROSGETCOLLISIONHANDLE"
   "<SIMROSGETCOLLISIONHANDLE-REQUEST>"
   "SIMROSGETCOLLISIONHANDLE-REQUEST"
   "<SIMROSGETCOLLISIONHANDLE-RESPONSE>"
   "SIMROSGETCOLLISIONHANDLE-RESPONSE"
   "SIMROSGETDIALOGINPUT"
   "<SIMROSGETDIALOGINPUT-REQUEST>"
   "SIMROSGETDIALOGINPUT-REQUEST"
   "<SIMROSGETDIALOGINPUT-RESPONSE>"
   "SIMROSGETDIALOGINPUT-RESPONSE"
   "SIMROSGETDIALOGRESULT"
   "<SIMROSGETDIALOGRESULT-REQUEST>"
   "SIMROSGETDIALOGRESULT-REQUEST"
   "<SIMROSGETDIALOGRESULT-RESPONSE>"
   "SIMROSGETDIALOGRESULT-RESPONSE"
   "SIMROSGETDISTANCEHANDLE"
   "<SIMROSGETDISTANCEHANDLE-REQUEST>"
   "SIMROSGETDISTANCEHANDLE-REQUEST"
   "<SIMROSGETDISTANCEHANDLE-RESPONSE>"
   "SIMROSGETDISTANCEHANDLE-RESPONSE"
   "SIMROSGETFLOATSIGNAL"
   "<SIMROSGETFLOATSIGNAL-REQUEST>"
   "SIMROSGETFLOATSIGNAL-REQUEST"
   "<SIMROSGETFLOATSIGNAL-RESPONSE>"
   "SIMROSGETFLOATSIGNAL-RESPONSE"
   "SIMROSGETFLOATINGPARAMETER"
   "<SIMROSGETFLOATINGPARAMETER-REQUEST>"
   "SIMROSGETFLOATINGPARAMETER-REQUEST"
   "<SIMROSGETFLOATINGPARAMETER-RESPONSE>"
   "SIMROSGETFLOATINGPARAMETER-RESPONSE"
   "SIMROSGETINFO"
   "<SIMROSGETINFO-REQUEST>"
   "SIMROSGETINFO-REQUEST"
   "<SIMROSGETINFO-RESPONSE>"
   "SIMROSGETINFO-RESPONSE"
   "SIMROSGETINTEGERPARAMETER"
   "<SIMROSGETINTEGERPARAMETER-REQUEST>"
   "SIMROSGETINTEGERPARAMETER-REQUEST"
   "<SIMROSGETINTEGERPARAMETER-RESPONSE>"
   "SIMROSGETINTEGERPARAMETER-RESPONSE"
   "SIMROSGETINTEGERSIGNAL"
   "<SIMROSGETINTEGERSIGNAL-REQUEST>"
   "SIMROSGETINTEGERSIGNAL-REQUEST"
   "<SIMROSGETINTEGERSIGNAL-RESPONSE>"
   "SIMROSGETINTEGERSIGNAL-RESPONSE"
   "SIMROSGETJOINTMATRIX"
   "<SIMROSGETJOINTMATRIX-REQUEST>"
   "SIMROSGETJOINTMATRIX-REQUEST"
   "<SIMROSGETJOINTMATRIX-RESPONSE>"
   "SIMROSGETJOINTMATRIX-RESPONSE"
   "SIMROSGETJOINTSTATE"
   "<SIMROSGETJOINTSTATE-REQUEST>"
   "SIMROSGETJOINTSTATE-REQUEST"
   "<SIMROSGETJOINTSTATE-RESPONSE>"
   "SIMROSGETJOINTSTATE-RESPONSE"
   "SIMROSGETLASTERRORS"
   "<SIMROSGETLASTERRORS-REQUEST>"
   "SIMROSGETLASTERRORS-REQUEST"
   "<SIMROSGETLASTERRORS-RESPONSE>"
   "SIMROSGETLASTERRORS-RESPONSE"
   "SIMROSGETMODELPROPERTY"
   "<SIMROSGETMODELPROPERTY-REQUEST>"
   "SIMROSGETMODELPROPERTY-REQUEST"
   "<SIMROSGETMODELPROPERTY-RESPONSE>"
   "SIMROSGETMODELPROPERTY-RESPONSE"
   "SIMROSGETOBJECTCHILD"
   "<SIMROSGETOBJECTCHILD-REQUEST>"
   "SIMROSGETOBJECTCHILD-REQUEST"
   "<SIMROSGETOBJECTCHILD-RESPONSE>"
   "SIMROSGETOBJECTCHILD-RESPONSE"
   "SIMROSGETOBJECTFLOATPARAMETER"
   "<SIMROSGETOBJECTFLOATPARAMETER-REQUEST>"
   "SIMROSGETOBJECTFLOATPARAMETER-REQUEST"
   "<SIMROSGETOBJECTFLOATPARAMETER-RESPONSE>"
   "SIMROSGETOBJECTFLOATPARAMETER-RESPONSE"
   "SIMROSGETOBJECTGROUPDATA"
   "<SIMROSGETOBJECTGROUPDATA-REQUEST>"
   "SIMROSGETOBJECTGROUPDATA-REQUEST"
   "<SIMROSGETOBJECTGROUPDATA-RESPONSE>"
   "SIMROSGETOBJECTGROUPDATA-RESPONSE"
   "SIMROSGETOBJECTHANDLE"
   "<SIMROSGETOBJECTHANDLE-REQUEST>"
   "SIMROSGETOBJECTHANDLE-REQUEST"
   "<SIMROSGETOBJECTHANDLE-RESPONSE>"
   "SIMROSGETOBJECTHANDLE-RESPONSE"
   "SIMROSGETOBJECTINTPARAMETER"
   "<SIMROSGETOBJECTINTPARAMETER-REQUEST>"
   "SIMROSGETOBJECTINTPARAMETER-REQUEST"
   "<SIMROSGETOBJECTINTPARAMETER-RESPONSE>"
   "SIMROSGETOBJECTINTPARAMETER-RESPONSE"
   "SIMROSGETOBJECTPARENT"
   "<SIMROSGETOBJECTPARENT-REQUEST>"
   "SIMROSGETOBJECTPARENT-REQUEST"
   "<SIMROSGETOBJECTPARENT-RESPONSE>"
   "SIMROSGETOBJECTPARENT-RESPONSE"
   "SIMROSGETOBJECTPOSE"
   "<SIMROSGETOBJECTPOSE-REQUEST>"
   "SIMROSGETOBJECTPOSE-REQUEST"
   "<SIMROSGETOBJECTPOSE-RESPONSE>"
   "SIMROSGETOBJECTPOSE-RESPONSE"
   "SIMROSGETOBJECTSELECTION"
   "<SIMROSGETOBJECTSELECTION-REQUEST>"
   "SIMROSGETOBJECTSELECTION-REQUEST"
   "<SIMROSGETOBJECTSELECTION-RESPONSE>"
   "SIMROSGETOBJECTSELECTION-RESPONSE"
   "SIMROSGETOBJECTS"
   "<SIMROSGETOBJECTS-REQUEST>"
   "SIMROSGETOBJECTS-REQUEST"
   "<SIMROSGETOBJECTS-RESPONSE>"
   "SIMROSGETOBJECTS-RESPONSE"
   "SIMROSGETSTRINGPARAMETER"
   "<SIMROSGETSTRINGPARAMETER-REQUEST>"
   "SIMROSGETSTRINGPARAMETER-REQUEST"
   "<SIMROSGETSTRINGPARAMETER-RESPONSE>"
   "SIMROSGETSTRINGPARAMETER-RESPONSE"
   "SIMROSGETSTRINGSIGNAL"
   "<SIMROSGETSTRINGSIGNAL-REQUEST>"
   "SIMROSGETSTRINGSIGNAL-REQUEST"
   "<SIMROSGETSTRINGSIGNAL-RESPONSE>"
   "SIMROSGETSTRINGSIGNAL-RESPONSE"
   "SIMROSGETUIBUTTONPROPERTY"
   "<SIMROSGETUIBUTTONPROPERTY-REQUEST>"
   "SIMROSGETUIBUTTONPROPERTY-REQUEST"
   "<SIMROSGETUIBUTTONPROPERTY-RESPONSE>"
   "SIMROSGETUIBUTTONPROPERTY-RESPONSE"
   "SIMROSGETUIEVENTBUTTON"
   "<SIMROSGETUIEVENTBUTTON-REQUEST>"
   "SIMROSGETUIEVENTBUTTON-REQUEST"
   "<SIMROSGETUIEVENTBUTTON-RESPONSE>"
   "SIMROSGETUIEVENTBUTTON-RESPONSE"
   "SIMROSGETUIHANDLE"
   "<SIMROSGETUIHANDLE-REQUEST>"
   "SIMROSGETUIHANDLE-REQUEST"
   "<SIMROSGETUIHANDLE-RESPONSE>"
   "SIMROSGETUIHANDLE-RESPONSE"
   "SIMROSGETUISLIDER"
   "<SIMROSGETUISLIDER-REQUEST>"
   "SIMROSGETUISLIDER-REQUEST"
   "<SIMROSGETUISLIDER-RESPONSE>"
   "SIMROSGETUISLIDER-RESPONSE"
   "SIMROSGETVISIONSENSORDEPTHBUFFER"
   "<SIMROSGETVISIONSENSORDEPTHBUFFER-REQUEST>"
   "SIMROSGETVISIONSENSORDEPTHBUFFER-REQUEST"
   "<SIMROSGETVISIONSENSORDEPTHBUFFER-RESPONSE>"
   "SIMROSGETVISIONSENSORDEPTHBUFFER-RESPONSE"
   "SIMROSGETVISIONSENSORIMAGE"
   "<SIMROSGETVISIONSENSORIMAGE-REQUEST>"
   "SIMROSGETVISIONSENSORIMAGE-REQUEST"
   "<SIMROSGETVISIONSENSORIMAGE-RESPONSE>"
   "SIMROSGETVISIONSENSORIMAGE-RESPONSE"
   "SIMROSLOADMODEL"
   "<SIMROSLOADMODEL-REQUEST>"
   "SIMROSLOADMODEL-REQUEST"
   "<SIMROSLOADMODEL-RESPONSE>"
   "SIMROSLOADMODEL-RESPONSE"
   "SIMROSLOADSCENE"
   "<SIMROSLOADSCENE-REQUEST>"
   "SIMROSLOADSCENE-REQUEST"
   "<SIMROSLOADSCENE-RESPONSE>"
   "SIMROSLOADSCENE-RESPONSE"
   "SIMROSLOADUI"
   "<SIMROSLOADUI-REQUEST>"
   "SIMROSLOADUI-REQUEST"
   "<SIMROSLOADUI-RESPONSE>"
   "SIMROSLOADUI-RESPONSE"
   "SIMROSPAUSESIMULATION"
   "<SIMROSPAUSESIMULATION-REQUEST>"
   "SIMROSPAUSESIMULATION-REQUEST"
   "<SIMROSPAUSESIMULATION-RESPONSE>"
   "SIMROSPAUSESIMULATION-RESPONSE"
   "SIMROSREADCOLLISION"
   "<SIMROSREADCOLLISION-REQUEST>"
   "SIMROSREADCOLLISION-REQUEST"
   "<SIMROSREADCOLLISION-RESPONSE>"
   "SIMROSREADCOLLISION-RESPONSE"
   "SIMROSREADDISTANCE"
   "<SIMROSREADDISTANCE-REQUEST>"
   "SIMROSREADDISTANCE-REQUEST"
   "<SIMROSREADDISTANCE-RESPONSE>"
   "SIMROSREADDISTANCE-RESPONSE"
   "SIMROSREADFORCESENSOR"
   "<SIMROSREADFORCESENSOR-REQUEST>"
   "SIMROSREADFORCESENSOR-REQUEST"
   "<SIMROSREADFORCESENSOR-RESPONSE>"
   "SIMROSREADFORCESENSOR-RESPONSE"
   "SIMROSREADPROXIMITYSENSOR"
   "<SIMROSREADPROXIMITYSENSOR-REQUEST>"
   "SIMROSREADPROXIMITYSENSOR-REQUEST"
   "<SIMROSREADPROXIMITYSENSOR-RESPONSE>"
   "SIMROSREADPROXIMITYSENSOR-RESPONSE"
   "SIMROSREADVISIONSENSOR"
   "<SIMROSREADVISIONSENSOR-REQUEST>"
   "SIMROSREADVISIONSENSOR-REQUEST"
   "<SIMROSREADVISIONSENSOR-RESPONSE>"
   "SIMROSREADVISIONSENSOR-RESPONSE"
   "SIMROSREMOVEMODEL"
   "<SIMROSREMOVEMODEL-REQUEST>"
   "SIMROSREMOVEMODEL-REQUEST"
   "<SIMROSREMOVEMODEL-RESPONSE>"
   "SIMROSREMOVEMODEL-RESPONSE"
   "SIMROSREMOVEOBJECT"
   "<SIMROSREMOVEOBJECT-REQUEST>"
   "SIMROSREMOVEOBJECT-REQUEST"
   "<SIMROSREMOVEOBJECT-RESPONSE>"
   "SIMROSREMOVEOBJECT-RESPONSE"
   "SIMROSREMOVEUI"
   "<SIMROSREMOVEUI-REQUEST>"
   "SIMROSREMOVEUI-REQUEST"
   "<SIMROSREMOVEUI-RESPONSE>"
   "SIMROSREMOVEUI-RESPONSE"
   "SIMROSSETARRAYPARAMETER"
   "<SIMROSSETARRAYPARAMETER-REQUEST>"
   "SIMROSSETARRAYPARAMETER-REQUEST"
   "<SIMROSSETARRAYPARAMETER-RESPONSE>"
   "SIMROSSETARRAYPARAMETER-RESPONSE"
   "SIMROSSETBOOLEANPARAMETER"
   "<SIMROSSETBOOLEANPARAMETER-REQUEST>"
   "SIMROSSETBOOLEANPARAMETER-REQUEST"
   "<SIMROSSETBOOLEANPARAMETER-RESPONSE>"
   "SIMROSSETBOOLEANPARAMETER-RESPONSE"
   "SIMROSSETFLOATSIGNAL"
   "<SIMROSSETFLOATSIGNAL-REQUEST>"
   "SIMROSSETFLOATSIGNAL-REQUEST"
   "<SIMROSSETFLOATSIGNAL-RESPONSE>"
   "SIMROSSETFLOATSIGNAL-RESPONSE"
   "SIMROSSETFLOATINGPARAMETER"
   "<SIMROSSETFLOATINGPARAMETER-REQUEST>"
   "SIMROSSETFLOATINGPARAMETER-REQUEST"
   "<SIMROSSETFLOATINGPARAMETER-RESPONSE>"
   "SIMROSSETFLOATINGPARAMETER-RESPONSE"
   "SIMROSSETINTEGERPARAMETER"
   "<SIMROSSETINTEGERPARAMETER-REQUEST>"
   "SIMROSSETINTEGERPARAMETER-REQUEST"
   "<SIMROSSETINTEGERPARAMETER-RESPONSE>"
   "SIMROSSETINTEGERPARAMETER-RESPONSE"
   "SIMROSSETINTEGERSIGNAL"
   "<SIMROSSETINTEGERSIGNAL-REQUEST>"
   "SIMROSSETINTEGERSIGNAL-REQUEST"
   "<SIMROSSETINTEGERSIGNAL-RESPONSE>"
   "SIMROSSETINTEGERSIGNAL-RESPONSE"
   "SIMROSSETJOINTFORCE"
   "<SIMROSSETJOINTFORCE-REQUEST>"
   "SIMROSSETJOINTFORCE-REQUEST"
   "<SIMROSSETJOINTFORCE-RESPONSE>"
   "SIMROSSETJOINTFORCE-RESPONSE"
   "SIMROSSETJOINTPOSITION"
   "<SIMROSSETJOINTPOSITION-REQUEST>"
   "SIMROSSETJOINTPOSITION-REQUEST"
   "<SIMROSSETJOINTPOSITION-RESPONSE>"
   "SIMROSSETJOINTPOSITION-RESPONSE"
   "SIMROSSETJOINTSTATE"
   "<SIMROSSETJOINTSTATE-REQUEST>"
   "SIMROSSETJOINTSTATE-REQUEST"
   "<SIMROSSETJOINTSTATE-RESPONSE>"
   "SIMROSSETJOINTSTATE-RESPONSE"
   "SIMROSSETJOINTTARGETPOSITION"
   "<SIMROSSETJOINTTARGETPOSITION-REQUEST>"
   "SIMROSSETJOINTTARGETPOSITION-REQUEST"
   "<SIMROSSETJOINTTARGETPOSITION-RESPONSE>"
   "SIMROSSETJOINTTARGETPOSITION-RESPONSE"
   "SIMROSSETJOINTTARGETVELOCITY"
   "<SIMROSSETJOINTTARGETVELOCITY-REQUEST>"
   "SIMROSSETJOINTTARGETVELOCITY-REQUEST"
   "<SIMROSSETJOINTTARGETVELOCITY-RESPONSE>"
   "SIMROSSETJOINTTARGETVELOCITY-RESPONSE"
   "SIMROSSETMODELPROPERTY"
   "<SIMROSSETMODELPROPERTY-REQUEST>"
   "SIMROSSETMODELPROPERTY-REQUEST"
   "<SIMROSSETMODELPROPERTY-RESPONSE>"
   "SIMROSSETMODELPROPERTY-RESPONSE"
   "SIMROSSETOBJECTFLOATPARAMETER"
   "<SIMROSSETOBJECTFLOATPARAMETER-REQUEST>"
   "SIMROSSETOBJECTFLOATPARAMETER-REQUEST"
   "<SIMROSSETOBJECTFLOATPARAMETER-RESPONSE>"
   "SIMROSSETOBJECTFLOATPARAMETER-RESPONSE"
   "SIMROSSETOBJECTINTPARAMETER"
   "<SIMROSSETOBJECTINTPARAMETER-REQUEST>"
   "SIMROSSETOBJECTINTPARAMETER-REQUEST"
   "<SIMROSSETOBJECTINTPARAMETER-RESPONSE>"
   "SIMROSSETOBJECTINTPARAMETER-RESPONSE"
   "SIMROSSETOBJECTPARENT"
   "<SIMROSSETOBJECTPARENT-REQUEST>"
   "SIMROSSETOBJECTPARENT-REQUEST"
   "<SIMROSSETOBJECTPARENT-RESPONSE>"
   "SIMROSSETOBJECTPARENT-RESPONSE"
   "SIMROSSETOBJECTPOSE"
   "<SIMROSSETOBJECTPOSE-REQUEST>"
   "SIMROSSETOBJECTPOSE-REQUEST"
   "<SIMROSSETOBJECTPOSE-RESPONSE>"
   "SIMROSSETOBJECTPOSE-RESPONSE"
   "SIMROSSETOBJECTPOSITION"
   "<SIMROSSETOBJECTPOSITION-REQUEST>"
   "SIMROSSETOBJECTPOSITION-REQUEST"
   "<SIMROSSETOBJECTPOSITION-RESPONSE>"
   "SIMROSSETOBJECTPOSITION-RESPONSE"
   "SIMROSSETOBJECTQUATERNION"
   "<SIMROSSETOBJECTQUATERNION-REQUEST>"
   "SIMROSSETOBJECTQUATERNION-REQUEST"
   "<SIMROSSETOBJECTQUATERNION-RESPONSE>"
   "SIMROSSETOBJECTQUATERNION-RESPONSE"
   "SIMROSSETOBJECTSELECTION"
   "<SIMROSSETOBJECTSELECTION-REQUEST>"
   "SIMROSSETOBJECTSELECTION-REQUEST"
   "<SIMROSSETOBJECTSELECTION-RESPONSE>"
   "SIMROSSETOBJECTSELECTION-RESPONSE"
   "SIMROSSETSPHERICALJOINTMATRIX"
   "<SIMROSSETSPHERICALJOINTMATRIX-REQUEST>"
   "SIMROSSETSPHERICALJOINTMATRIX-REQUEST"
   "<SIMROSSETSPHERICALJOINTMATRIX-RESPONSE>"
   "SIMROSSETSPHERICALJOINTMATRIX-RESPONSE"
   "SIMROSSETSTRINGSIGNAL"
   "<SIMROSSETSTRINGSIGNAL-REQUEST>"
   "SIMROSSETSTRINGSIGNAL-REQUEST"
   "<SIMROSSETSTRINGSIGNAL-RESPONSE>"
   "SIMROSSETSTRINGSIGNAL-RESPONSE"
   "SIMROSSETUIBUTTONLABEL"
   "<SIMROSSETUIBUTTONLABEL-REQUEST>"
   "SIMROSSETUIBUTTONLABEL-REQUEST"
   "<SIMROSSETUIBUTTONLABEL-RESPONSE>"
   "SIMROSSETUIBUTTONLABEL-RESPONSE"
   "SIMROSSETUIBUTTONPROPERTY"
   "<SIMROSSETUIBUTTONPROPERTY-REQUEST>"
   "SIMROSSETUIBUTTONPROPERTY-REQUEST"
   "<SIMROSSETUIBUTTONPROPERTY-RESPONSE>"
   "SIMROSSETUIBUTTONPROPERTY-RESPONSE"
   "SIMROSSETUISLIDER"
   "<SIMROSSETUISLIDER-REQUEST>"
   "SIMROSSETUISLIDER-REQUEST"
   "<SIMROSSETUISLIDER-RESPONSE>"
   "SIMROSSETUISLIDER-RESPONSE"
   "SIMROSSETVISIONSENSORIMAGE"
   "<SIMROSSETVISIONSENSORIMAGE-REQUEST>"
   "SIMROSSETVISIONSENSORIMAGE-REQUEST"
   "<SIMROSSETVISIONSENSORIMAGE-RESPONSE>"
   "SIMROSSETVISIONSENSORIMAGE-RESPONSE"
   "SIMROSSTARTSIMULATION"
   "<SIMROSSTARTSIMULATION-REQUEST>"
   "SIMROSSTARTSIMULATION-REQUEST"
   "<SIMROSSTARTSIMULATION-RESPONSE>"
   "SIMROSSTARTSIMULATION-RESPONSE"
   "SIMROSSTOPSIMULATION"
   "<SIMROSSTOPSIMULATION-REQUEST>"
   "SIMROSSTOPSIMULATION-REQUEST"
   "<SIMROSSTOPSIMULATION-RESPONSE>"
   "SIMROSSTOPSIMULATION-RESPONSE"
   "SIMROSSYNCHRONOUS"
   "<SIMROSSYNCHRONOUS-REQUEST>"
   "SIMROSSYNCHRONOUS-REQUEST"
   "<SIMROSSYNCHRONOUS-RESPONSE>"
   "SIMROSSYNCHRONOUS-RESPONSE"
   "SIMROSSYNCHRONOUSTRIGGER"
   "<SIMROSSYNCHRONOUSTRIGGER-REQUEST>"
   "SIMROSSYNCHRONOUSTRIGGER-REQUEST"
   "<SIMROSSYNCHRONOUSTRIGGER-RESPONSE>"
   "SIMROSSYNCHRONOUSTRIGGER-RESPONSE"
   "SIMROSTRANSFERFILE"
   "<SIMROSTRANSFERFILE-REQUEST>"
   "SIMROSTRANSFERFILE-REQUEST"
   "<SIMROSTRANSFERFILE-RESPONSE>"
   "SIMROSTRANSFERFILE-RESPONSE"
  ))

