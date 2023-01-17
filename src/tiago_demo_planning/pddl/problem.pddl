(define (problem task)
    (:domain tiago_demo)
    (:objects
        table1 table2 table3 table4 table5 table - waypoint
        tiago - robot
        glass - ball
    )
    (:init
        (robot-at tiago table)
        (not (ball-at glass table))
        (connected table table1)
        (connected table1 table2)
        (connected table2 table3)
        (connected table table4)
        (connected table4 table5)
        (connected table1 table)
        (connected table2 table1)
        (connected table3 table2)
        (connected table4 table)
        (connected table5 table4)
    )
    (:goal
        (and
            (ball-at glass table)
        )
    )
)