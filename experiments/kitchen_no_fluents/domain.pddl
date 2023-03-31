(define (domain kitchen)
    (:requirements :strips :derived-predicates :disjunctive-preconditions :equality :negative-preconditions)
    (:predicates 
        ;static predicates
        (item ?item)
        (region ?region)
        (sink ?region)
        (burner ?region)
        (cooked ?item)
        (clean ?item)
        (traj ?traj)
        ; I: item frame, W: world frame, H: hand frame
        (worldpose ?item ?X_WI)
        (handpose ?item ?X_HI)
        ;7 DOF conf 
        (conf ?q)
        (graspconf ?q)
        ;sequence of 7 DOF confs, linearly interpolated
        ; if item where at X_WI, would it be in region?
        (contained ?item ?X_WI ?region)
        (motion ?q1 ?traj ?q2)
        (ik ?item ?X_WI ?X_HI ?pre_q ?q)
        (colfree ?q ?item ?X_WI)
        (colfree-freetraj ?traj ?item ?X_WI)
        (colfree-holdingtraj ?traj ?holdingitem ?X_HI ?otheritem ?X_WI)

        ;fluents predicates
        (atconf ?q)
        (atpose ?item ?X_WI)
        (holding ?item ?X_HI)
        (empty)

        ;derived
        (in ?item ?region)
        (safe ?q ?item)
        (safe-freetraj ?traj ?item)
        (safe-holdingtraj ?traj ?holdingitem ?X_HI ?otheritem)
        ;(safeplace ?q ?itemholding ?X_HI ?item)
    )

    ;(:functions
        ;(distance ?traj)
    ;)

    (:derived (safe ?q ?item) 
        (or
            (exists (?X_HI)
                (and
                    (holding ?item ?X_HI)
                    (handpose ?item ?X_HI)
                )
            ) 
            (exists (?X_WI)
                (and
                    (colfree ?q ?item ?X_WI)
                    (atpose ?item ?X_WI) 
                ) 
            ) 
        )
    )

    (:derived (in ?item ?region) 
        ; does there exist a pose ?X_WI such that ?item is at ?X_WI and 
        ; if it were it would be contained within ?region 
        (exists (?X_WI) (and
                (contained ?item ?X_WI ?region) 
                (atpose ?item ?X_WI)
            )
        )
    )

    (:derived (safe-freetraj ?traj ?item)
        (exists (?X_WI) (and
                (worldpose ?item ?X_WI)
                (atpose ?item ?X_WI)
                (colfree-freetraj ?traj ?item ?X_WI) 
            )
        ) 
    )

    (:derived (safe-holdingtraj ?traj ?holdingitem ?X_HI ?otheritem)
        (exists (?X_WI) (and
                (worldpose ?otheritem ?X_WI)
                (atpose ?otheritem ?X_WI)
                (colfree-holdingtraj ?traj ?holdingitem ?X_HI ?otheritem ?X_WI) 
            )
        ) 
    )

    (:action move-free
        :parameters(?q1 ?traj ?q2) 
        :precondition (and 
            (motion ?q1 ?traj ?q2)
            (empty)
            (atconf ?q1)
            (forall (?item)
                (imply
                    (item ?item) 
                    (safe-freetraj ?traj ?item)
                ) 
            )
        )
        :effect (and 
            (atconf ?q2)
            (not (atconf ?q1))
            ;(increase (total-cost) (distance ?traj)) 
            ; TODO(agro): add cost here
        )
    )

    (:action move-holding
        :parameters(?q1 ?traj ?q2 ?holdingitem ?X_HI) 
        :precondition (and 
            (handpose ?holdingitem ?X_HI)
            (not (empty))
            (motion ?q1 ?traj ?q2)
            (atconf ?q1)
            (forall (?otheritem)
                (or
                    (= ?otheritem ?holdingitem) 
                    (imply
                        (item ?otheritem) 
                        (safe-holdingtraj ?traj ?holdingitem ?X_HI ?otheritem)
                    ) 
                )
            )
        )
        :effect (and 
            (atconf ?q2)
            (not (atconf ?q1))
            ;(increase (total-cost) (distance ?traj)) 
            ; TODO(agro): add cost here
        )
    )

    (:action pick
        :parameters (?item ?X_WI ?X_HI ?pre_q ?q)
        :precondition (and 
            (ik ?item ?X_WI ?X_HI ?pre_q ?q)
            (atpose ?item ?X_WI)
            (empty)
            (atconf ?pre_q)
            (forall (?otheritem)
                (imply 
                    (item ?otheritem) 
                    (safe ?q ?otheritem)
                ) 
            )
        )
        :effect (and
            (holding ?item ?X_HI)
            (not (atpose ?item ?X_WI))
            (not (empty))
        )
    )

    (:action place
        :parameters (?item ?X_WI ?X_HI ?pre_q ?q)
        :precondition (and 
            (ik ?item ?X_WI ?X_HI ?pre_q ?q)
            (holding ?item ?X_HI)
            (atconf ?pre_q)
            (forall (?otheritem)
                (imply 
                    (item ?otheritem) 
                    (safe ?q ?otheritem)
                ) 
            )
        )
        :effect (and
            (not (holding ?item ?X_HI))
            (atpose ?item ?X_WI) 
            (empty)
        )    
    )

    (:action wash
        :parameters (?item ?region) 
        :precondition (and
            (region ?region)
            (item ?item) 
            (sink ?region) 
            (in ?item ?region) 
        )
        :effect (and
            (clean ?item)
        )
    )

    (:action cook
        :parameters (?item ?region) 
        :precondition (and 
            (item ?item)
            (region ?region)
            (burner ?region)
            (clean ?item)
            (in ?item ?region)
        )
        :effect (and
            (cooked ?item)
        )
    )


)