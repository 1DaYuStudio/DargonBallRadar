idf_component_register(
                        SRCS  "main.c" 
                              "LCD_Driver/ST7701S.c" 
                              "test2.c"
                              "ball.c"

                        INCLUDE_DIRS "./LCD_Driver"
                                     "."
                        )
