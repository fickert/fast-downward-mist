(define (problem logistics-9-0)
(:domain logistics)
(:objects apn1 apt3 pos3 apt2 pos2 apt1 pos1 cit3 cit2 cit1 tru3 tru2 tru1 obj33 obj32 obj31 obj23 obj22 obj21 obj13 obj12 obj11 )
(:init (package obj11) (package obj12) (package obj13) (package obj21)
 (package obj22) (package obj23) (package obj31) (package obj32) (package obj33)
 (truck tru1) (truck tru2) (truck tru3) (city cit1) (city cit2) (city cit3)
 (location pos1) (location apt1) (location pos2) (location apt2) (location pos3)
 (location apt3) (airport apt1) (airport apt2) (airport apt3) (airplane apn1)
 (at apn1 apt2) (at tru1 pos1) (at obj11 pos1) (at obj12 pos1) (at obj13 pos1)
 (at tru2 pos2) (at obj21 pos2) (at obj22 pos2) (at obj23 pos2) (at tru3 pos3)
 (at obj31 pos3) (at obj32 pos3) (at obj33 pos3) (in-city pos1 cit1)
 (in-city apt1 cit1) (in-city pos2 cit2) (in-city apt2 cit2) (in-city pos3 cit3)
 (in-city apt3 cit3))
(:goal (and
	(at obj21 apt1)
	(at obj12 pos1)
	(at obj11 pos1)
	(at obj22 pos1)
	(at obj31 apt3)
)
)
(:onlinegoals
(and
	(at obj13 apt3)
	(at obj32 pos1)
	(at obj23 pos3)
	(at obj33 pos3)
)
	(2423)
)
(:execution_time 1381)

)
