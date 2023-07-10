(define (problem task)
    (:domain tiago_demo)
    (:objects
        table1 table2 table4 initial - waypoint
        tiago - robot
        glass - ball
    )
    (:init
        (robot-at tiago initial)
        (not (ball-at glass initial))
        (connected initial table1)
        (connected table1 table2)
        (connected initial table4)
        (connected table1 initial)
        (connected table2 table1)
        (connected table4 initial)
    )
    (:goal
        (and
            (ball-at glass initial)
        )
    )
)