/*
 feature_vector[0] - Core
feature_vector[1] - RPS

output_index[0] - FDS_QSPINLOCK
output_index[1] - FDS_TCLOCK
output_index[2] - FDS_TDLOCK
*/
// Tree 0
int predict_tree_0(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 18750) {
        if (features[1] <= 11250) {
          if (features[0] <= 121) {
            if (features[0] <= 67) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 94) {
                if (features[1] <= 8750) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                if (features[1] <= 6250) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 135) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 202) {
          if (features[0] <= 162) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 52500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 37500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[1] <= 187500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 562500) {
            if (features[1] <= 437500) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[0] <= 15) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 94) {
        if (features[1] <= 312500) {
          if (features[0] <= 67) {
            if (features[1] <= 187500) {
              if (features[1] <= 107500) {
                if (features[1] <= 85000) {
                  return 1; // FDS_TCLOCK
                } else {
                  if (features[0] <= 40) {
                    return 0; // FDS_QSPINLOCK
                  } else {
                    return 1; // FDS_TCLOCK
                  }
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 40) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            if (features[1] <= 95000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 1
int predict_tree_1(int features[]) {
  if (features[1] <= 47500) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 18750) {
        if (features[1] <= 11250) {
          if (features[0] <= 148) {
            if (features[0] <= 67) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 94) {
                if (features[1] <= 8750) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                if (features[1] <= 6250) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 0; // FDS_QSPINLOCK
        }
      } else {
        return 1; // FDS_TCLOCK
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[1] <= 95000) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 148) {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 75000) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 67) {
          if (features[0] <= 37) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[0] <= 40) {
          if (features[1] <= 437500) {
            if (features[0] <= 7) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 18) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 15) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 2
int predict_tree_2(int features[]) {
  if (features[1] <= 95000) {
    if (features[1] <= 8750) {
      if (features[0] <= 148) {
        if (features[0] <= 94) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 121) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 0; // FDS_QSPINLOCK
          }
        }
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 65000) {
        if (features[1] <= 47500) {
          if (features[1] <= 30000) {
            if (features[1] <= 18750) {
              if (features[1] <= 11250) {
                if (features[0] <= 50) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                if (features[0] <= 67) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            } else {
              if (features[0] <= 40) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            if (features[0] <= 67) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 55000) {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      } else {
        if (features[1] <= 85000) {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 81) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 108) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 18) {
      if (features[1] <= 250000) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 562500) {
          if (features[1] <= 437500) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 9) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 0; // FDS_QSPINLOCK
        }
      }
    } else {
      if (features[1] <= 312500) {
        if (features[0] <= 67) {
          if (features[1] <= 112500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 187500) {
              if (features[0] <= 37) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 40) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 3
int predict_tree_3(int features[]) {
  if (features[1] <= 95000) {
    if (features[1] <= 47500) {
      if (features[1] <= 8750) {
        if (features[0] <= 108) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 11250) {
          if (features[0] <= 67) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 30000) {
            if (features[1] <= 18750) {
              if (features[0] <= 50) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 40) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            if (features[0] <= 37) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 75000) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 85000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 121) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 437500) {
      if (features[0] <= 18) {
        if (features[0] <= 9) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 13) {
            if (features[1] <= 250000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 0; // FDS_QSPINLOCK
          }
        }
      } else {
        if (features[1] <= 187500) {
          if (features[0] <= 67) {
            if (features[0] <= 37) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[0] <= 18) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 15) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 4
int predict_tree_4(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 202) {
        if (features[0] <= 148) {
          if (features[0] <= 94) {
            if (features[0] <= 67) {
              if (features[1] <= 27500) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 121) {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 7500) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          }
        } else {
          if (features[1] <= 37500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 30000) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 562500) {
      if (features[0] <= 67) {
        if (features[1] <= 187500) {
          if (features[0] <= 23) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 85000) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 107500) {
                if (features[0] <= 40) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          }
        } else {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 437500) {
                if (features[0] <= 15) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                if (features[0] <= 15) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          }
        }
      } else {
        if (features[1] <= 95000) {
          if (features[1] <= 85000) {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 5
int predict_tree_5(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 312500) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 187500) {
          if (features[0] <= 18) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 15) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 15) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 85000) {
      if (features[0] <= 121) {
        if (features[0] <= 67) {
          if (features[1] <= 18750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 94) {
            if (features[1] <= 8750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 7500) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 70000) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      } else {
        if (features[1] <= 55000) {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 6
int predict_tree_6(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 112500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 187500) {
        if (features[0] <= 23) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 562500) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 312500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 94) {
      if (features[0] <= 67) {
        if (features[1] <= 23750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 107500) {
          if (features[1] <= 27500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 175) {
        if (features[0] <= 148) {
          if (features[0] <= 121) {
            if (features[1] <= 70000) {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 62500) {
              if (features[1] <= 7500) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 37500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 202) {
          if (features[1] <= 57500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 31250) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 7
int predict_tree_7(int features[]) {
  if (features[1] <= 75000) {
    if (features[1] <= 47500) {
      if (features[0] <= 67) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 16250) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 121) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 7500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 65000) {
          if (features[0] <= 54) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 162) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 135) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[0] <= 23) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 95000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 18) {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 40) {
          if (features[0] <= 23) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 8
int predict_tree_8(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 148) {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            if (features[1] <= 16250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 7500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 121) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[1] <= 47500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 112500) {
      if (features[1] <= 95000) {
        if (features[1] <= 85000) {
          if (features[0] <= 94) {
            if (features[0] <= 37) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 77) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[1] <= 437500) {
          if (features[1] <= 312500) {
            if (features[0] <= 67) {
              if (features[0] <= 12) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[1] <= 187500) {
                  return 1; // FDS_TCLOCK
                } else {
                  if (features[0] <= 40) {
                    return 1; // FDS_TCLOCK
                  } else {
                    return 2; // FDS_TDLOCK
                  }
                }
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 48) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 15) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 9
int predict_tree_9(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 437500) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 23) {
          if (features[0] <= 15) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 187500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 15) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 94) {
      if (features[1] <= 107500) {
        if (features[0] <= 67) {
          if (features[1] <= 23750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 10000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 175) {
        if (features[1] <= 75000) {
          if (features[0] <= 148) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 202) {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 37500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 10
int predict_tree_10(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 47500) {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            if (features[1] <= 22500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 8750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 148) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 148) {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[0] <= 121) {
                if (features[1] <= 65000) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                if (features[1] <= 70000) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 37) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 112500) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[0] <= 121) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 312500) {
      if (features[0] <= 40) {
        if (features[0] <= 12) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[1] <= 562500) {
        if (features[1] <= 437500) {
          if (features[0] <= 12) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 21) {
            if (features[0] <= 9) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 18) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 11
int predict_tree_11(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 67) {
      if (features[1] <= 55000) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 47500) {
        if (features[0] <= 148) {
          if (features[1] <= 7500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 55000) {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 65000) {
            if (features[0] <= 162) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 112500) {
      if (features[1] <= 95000) {
        if (features[1] <= 85000) {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 108) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 50) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 48) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 562500) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 18) {
              if (features[1] <= 250000) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 23) {
                return 2; // FDS_TDLOCK
              } else {
                if (features[1] <= 312500) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          }
        } else {
          if (features[0] <= 18) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 67) {
          if (features[1] <= 250000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 12
int predict_tree_12(int features[]) {
  if (features[1] <= 85000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 8750) {
        if (features[1] <= 6250) {
          if (features[0] <= 162) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 94) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 47500) {
          if (features[0] <= 67) {
            if (features[1] <= 18750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 65000) {
            if (features[1] <= 55000) {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            if (features[0] <= 121) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[1] <= 112500) {
        if (features[1] <= 95000) {
          if (features[0] <= 94) {
            if (features[0] <= 40) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 50) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 81) {
          if (features[0] <= 18) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 437500) {
        if (features[1] <= 312500) {
          if (features[0] <= 40) {
            if (features[0] <= 15) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 18) {
            if (features[0] <= 7) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 14) {
          if (features[0] <= 5) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 13
int predict_tree_13(int features[]) {
  if (features[1] <= 47500) {
    if (features[1] <= 8750) {
      if (features[1] <= 6250) {
        if (features[0] <= 148) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 135) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 81) {
          if (features[1] <= 23750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[1] <= 95000) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 85000) {
            if (features[1] <= 65000) {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[1] <= 75000) {
                if (features[0] <= 135) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                if (features[0] <= 108) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          } else {
            if (features[0] <= 81) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 112500) {
          if (features[0] <= 77) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 23) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 81) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[0] <= 40) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 437500) {
                if (features[0] <= 15) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                if (features[0] <= 18) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 15) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 14
int predict_tree_14(int features[]) {
  if (features[1] <= 47500) {
    if (features[1] <= 40000) {
      if (features[1] <= 30000) {
        if (features[0] <= 94) {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 18750) {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[0] <= 67) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 148) {
            if (features[0] <= 121) {
              if (features[1] <= 6250) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 7500) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        return 0; // FDS_QSPINLOCK
      }
    } else {
      if (features[0] <= 37) {
        return 0; // FDS_QSPINLOCK
      } else {
        return 1; // FDS_TCLOCK
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 65000) {
          if (features[1] <= 55000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 162) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 94) {
            if (features[1] <= 102500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[1] <= 312500) {
          if (features[0] <= 37) {
            if (features[0] <= 12) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 21) {
            if (features[1] <= 437500) {
              if (features[0] <= 6) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 6) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 15) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 15
int predict_tree_15(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 47500) {
      if (features[0] <= 67) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 23750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 121) {
          if (features[0] <= 94) {
            if (features[1] <= 7500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 20000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 148) {
            if (features[1] <= 7500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[1] <= 75000) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 65000) {
            if (features[1] <= 55000) {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            if (features[0] <= 135) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 112500) {
          if (features[1] <= 85000) {
            if (features[0] <= 54) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 94) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            if (features[1] <= 95000) {
              if (features[0] <= 81) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 89) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[0] <= 18) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 54) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 437500) {
      if (features[0] <= 12) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 23) {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 35) {
        if (features[1] <= 562500) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 16
int predict_tree_16(int features[]) {
  if (features[1] <= 95000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 94) {
        if (features[0] <= 67) {
          if (features[1] <= 18750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 8750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 202) {
          if (features[1] <= 75000) {
            if (features[0] <= 175) {
              if (features[1] <= 6250) {
                if (features[0] <= 135) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 47500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 37500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[0] <= 23) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 112500) {
          return 2; // FDS_TDLOCK
        } else {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[1] <= 437500) {
          if (features[1] <= 312500) {
            if (features[0] <= 55) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 18) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 17
int predict_tree_17(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 6) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 187500) {
        if (features[1] <= 112500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 23) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 312500) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[1] <= 437500) {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 562500) {
              if (features[0] <= 18) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 18) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 175) {
      if (features[0] <= 148) {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            if (features[1] <= 250000) {
              if (features[1] <= 20000) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 90000) {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 70000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 52500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 202) {
        if (features[1] <= 37500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 42500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 18
int predict_tree_18(int features[]) {
  if (features[1] <= 75000) {
    if (features[1] <= 11250) {
      if (features[0] <= 94) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 148) {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 162) {
          if (features[1] <= 18750) {
            if (features[0] <= 67) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 202) {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[0] <= 37) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 112500) {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 562500) {
          if (features[1] <= 437500) {
            if (features[1] <= 312500) {
              if (features[0] <= 14) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 7) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 18) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[0] <= 15) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 19
int predict_tree_19(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 55000) {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 8750) {
          if (features[0] <= 94) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 135) {
              if (features[1] <= 6250) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 67) {
            if (features[1] <= 18750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[1] <= 75000) {
        if (features[0] <= 54) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 65000) {
            if (features[0] <= 162) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 54) {
          if (features[1] <= 112500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 23) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 95000) {
            if (features[0] <= 108) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[0] <= 7) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 437500) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[1] <= 562500) {
            if (features[0] <= 18) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[1] <= 312500) {
        if (features[0] <= 40) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 20
int predict_tree_20(int features[]) {
  if (features[1] <= 112500) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 148) {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 8750) {
              if (features[1] <= 6250) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[0] <= 94) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 37500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 95000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 65000) {
            if (features[0] <= 162) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[0] <= 67) {
        if (features[0] <= 32) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[1] <= 562500) {
        if (features[1] <= 437500) {
          if (features[0] <= 18) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 312500) {
              if (features[0] <= 40) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 14) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 18) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 21
int predict_tree_21(int features[]) {
  if (features[1] <= 85000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 47500) {
        if (features[1] <= 18750) {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 11250) {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 148) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 13) {
      if (features[0] <= 9) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 250000) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[0] <= 94) {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 67) {
          if (features[1] <= 187500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 18) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 312500) {
                if (features[0] <= 40) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 22
int predict_tree_22(int features[]) {
  if (features[1] <= 47500) {
    if (features[1] <= 11250) {
      if (features[0] <= 108) {
        return 0; // FDS_QSPINLOCK
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 67) {
          if (features[1] <= 18750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    }
  } else {
    if (features[1] <= 85000) {
      if (features[1] <= 75000) {
        if (features[0] <= 37) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 65000) {
            if (features[1] <= 55000) {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 135) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 108) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 112500) {
        if (features[1] <= 95000) {
          if (features[0] <= 64) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 91) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 312500) {
          if (features[1] <= 187500) {
            if (features[0] <= 67) {
              if (features[0] <= 23) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 67) {
              if (features[0] <= 12) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 562500) {
            if (features[0] <= 12) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 15) {
              if (features[0] <= 7) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  }
}

// Tree 23
int predict_tree_23(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 312500) {
        if (features[0] <= 12) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 18) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 95000) {
      if (features[0] <= 148) {
        if (features[1] <= 6250) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 67) {
            if (features[1] <= 17500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 75000) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 85000) {
                if (features[0] <= 108) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          }
        }
      } else {
        if (features[0] <= 202) {
          if (features[1] <= 42500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 24
int predict_tree_24(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 18) {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 312500) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 23) {
        if (features[1] <= 170000) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 375000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 107500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 121) {
      if (features[0] <= 67) {
        if (features[1] <= 18750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 250000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 75000) {
          if (features[1] <= 8750) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 94) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 175) {
        if (features[1] <= 52500) {
          if (features[0] <= 148) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 37500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 25
int predict_tree_25(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      if (features[0] <= 23) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 107500) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 18) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 15) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[1] <= 6250) {
        if (features[0] <= 162) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 175) {
          if (features[1] <= 18750) {
            if (features[1] <= 8750) {
              if (features[0] <= 108) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 11250) {
                return 1; // FDS_TCLOCK
              } else {
                if (features[0] <= 67) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[0] <= 94) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 26
int predict_tree_26(int features[]) {
  if (features[1] <= 187500) {
    if (features[0] <= 40) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 23) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 85000) {
        if (features[0] <= 175) {
          if (features[1] <= 18750) {
            if (features[1] <= 6250) {
              if (features[0] <= 148) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 81) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 94) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 437500) {
      if (features[1] <= 312500) {
        if (features[0] <= 40) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[1] <= 562500) {
        if (features[0] <= 32) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 15) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 27
int predict_tree_27(int features[]) {
  if (features[1] <= 75000) {
    if (features[1] <= 11250) {
      if (features[0] <= 148) {
        if (features[0] <= 81) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 40000) {
        if (features[1] <= 30000) {
          if (features[0] <= 67) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 37) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 55000) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 65000) {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 112500) {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 85000) {
          return 2; // FDS_TDLOCK
        } else {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 37) {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 15) {
            if (features[0] <= 9) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 250000) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 67) {
          if (features[1] <= 187500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 28
int predict_tree_28(int features[]) {
  if (features[1] <= 85000) {
    if (features[1] <= 47500) {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 6250) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 8750) {
            if (features[0] <= 81) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[1] <= 75000) {
        if (features[1] <= 55000) {
          if (features[0] <= 54) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 65000) {
            if (features[0] <= 40) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            if (features[0] <= 32) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 135) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      } else {
        if (features[0] <= 37) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[1] <= 437500) {
        if (features[0] <= 18) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 187500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 562500) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 187500) {
        if (features[1] <= 112500) {
          if (features[1] <= 95000) {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 54) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 29
int predict_tree_29(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 55000) {
      if (features[1] <= 11250) {
        if (features[1] <= 6250) {
          if (features[0] <= 135) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 94) {
            if (features[1] <= 8750) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 30000) {
          if (features[1] <= 18750) {
            if (features[0] <= 54) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 77) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 47500) {
            if (features[1] <= 40000) {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 40) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            if (features[0] <= 40) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      if (features[1] <= 75000) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 65000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 40) {
          if (features[1] <= 112500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 23) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 94) {
              if (features[1] <= 90000) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 562500) {
      if (features[0] <= 15) {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 40) {
          if (features[0] <= 23) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 18) {
        return 1; // FDS_TCLOCK
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 30
int predict_tree_30(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 23) {
      if (features[1] <= 312500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 562500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 102500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 250000) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 94) {
      if (features[0] <= 67) {
        if (features[1] <= 17500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 250000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 8750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 95000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 65000) {
        if (features[1] <= 6250) {
          if (features[0] <= 148) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 55000) {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 31
int predict_tree_31(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 18) {
      if (features[1] <= 250000) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 437500) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[1] <= 170000) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 312500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 148) {
      if (features[0] <= 67) {
        if (features[1] <= 18750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 250000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 75000) {
          if (features[0] <= 94) {
            if (features[1] <= 15000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 94) {
            if (features[1] <= 95000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 202) {
        if (features[0] <= 175) {
          if (features[1] <= 30000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 42500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 32
int predict_tree_32(int features[]) {
  if (features[1] <= 187500) {
    if (features[0] <= 40) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 18) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 47500) {
        if (features[0] <= 94) {
          if (features[1] <= 8750) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 67) {
              if (features[1] <= 17500) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 6250) {
            if (features[0] <= 135) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 65000) {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 85000) {
            if (features[0] <= 121) {
              if (features[0] <= 81) {
                return 1; // FDS_TCLOCK
              } else {
                if (features[1] <= 75000) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 562500) {
      if (features[0] <= 23) {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 15) {
        return 1; // FDS_TCLOCK
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 33
int predict_tree_33(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 47500) {
        if (features[0] <= 67) {
          if (features[1] <= 28750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 8750) {
            if (features[0] <= 94) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 148) {
                if (features[1] <= 6250) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 55000) {
          if (features[0] <= 162) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 65000) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[0] <= 9) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 13) {
          if (features[1] <= 250000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 0; // FDS_QSPINLOCK
        }
      }
    } else {
      if (features[0] <= 67) {
        if (features[0] <= 40) {
          if (features[1] <= 232500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 187500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 34
int predict_tree_34(int features[]) {
  if (features[1] <= 95000) {
    if (features[1] <= 47500) {
      if (features[0] <= 67) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 17500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 94) {
          if (features[1] <= 7500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 148) {
          if (features[1] <= 75000) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 85000) {
              if (features[0] <= 94) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 15) {
      if (features[1] <= 437500) {
        if (features[0] <= 9) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 67) {
        if (features[1] <= 312500) {
          if (features[1] <= 112500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 40) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 187500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 35
int predict_tree_35(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 65000) {
      if (features[1] <= 11250) {
        if (features[0] <= 94) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 47500) {
            if (features[0] <= 67) {
              if (features[1] <= 18750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 112500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 18) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 95000) {
          if (features[0] <= 121) {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 75000) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 112500) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[0] <= 67) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[1] <= 562500) {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 375000) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 18) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 36
int predict_tree_36(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 13) {
      if (features[1] <= 312500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 562500) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 23) {
        if (features[0] <= 18) {
          if (features[1] <= 300000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 175000) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 375000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 97500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 250000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 65000) {
      if (features[0] <= 67) {
        if (features[1] <= 23750) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 8750) {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 94) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 55000) {
              if (features[0] <= 135) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 175) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[0] <= 94) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 37
int predict_tree_37(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 18) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 437500) {
        if (features[1] <= 312500) {
          if (features[0] <= 12) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 15) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 18) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 47500) {
      if (features[0] <= 121) {
        if (features[1] <= 8750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 67) {
            if (features[1] <= 18750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 187500) {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 95000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 121) {
            if (features[1] <= 70000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 38
int predict_tree_38(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 6) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 250000) {
          if (features[0] <= 23) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 437500) {
            if (features[0] <= 18) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 562500) {
              if (features[0] <= 21) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 15) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[0] <= 67) {
        if (features[1] <= 28750) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 6250) {
          if (features[0] <= 162) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 47500) {
            if (features[1] <= 8750) {
              if (features[0] <= 94) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 65000) {
              if (features[1] <= 55000) {
                if (features[0] <= 148) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                if (features[0] <= 162) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            } else {
              if (features[0] <= 135) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[1] <= 85000) {
          if (features[0] <= 81) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 39
int predict_tree_39(int features[]) {
  if (features[1] <= 112500) {
    if (features[1] <= 75000) {
      if (features[1] <= 8750) {
        if (features[0] <= 162) {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 94) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 55000) {
          if (features[1] <= 47500) {
            if (features[0] <= 67) {
              if (features[1] <= 18750) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[0] <= 40) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 54) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[0] <= 37) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 65000) {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[1] <= 85000) {
          if (features[0] <= 37) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 135) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 81) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 50) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[0] <= 81) {
        return 1; // FDS_TCLOCK
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 562500) {
          if (features[1] <= 312500) {
            if (features[0] <= 12) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 7) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 437500) {
                if (features[0] <= 15) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          }
        } else {
          return 0; // FDS_QSPINLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 40
int predict_tree_40(int features[]) {
  if (features[1] <= 85000) {
    if (features[1] <= 40000) {
      if (features[0] <= 67) {
        if (features[1] <= 30000) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 94) {
          if (features[1] <= 10000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 148) {
          if (features[1] <= 75000) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 202) {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 312500) {
      if (features[1] <= 112500) {
        if (features[0] <= 54) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 23) {
          if (features[0] <= 12) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 187500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 187500) {
            if (features[0] <= 67) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 40) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[0] <= 15) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 41
int predict_tree_41(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 112500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 15) {
          if (features[0] <= 9) {
            return 1; // FDS_TCLOCK
          } else {
            return 0; // FDS_QSPINLOCK
          }
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[0] <= 94) {
        if (features[0] <= 67) {
          if (features[1] <= 17500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 16250) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 202) {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 41250) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[1] <= 85000) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 42
int predict_tree_42(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 562500) {
          if (features[1] <= 437500) {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[0] <= 15) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 15) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 94) {
      if (features[0] <= 67) {
        if (features[1] <= 28750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 170000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 102500) {
          if (features[1] <= 8750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 47500) {
        if (features[1] <= 6250) {
          if (features[0] <= 148) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 65000) {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 43
int predict_tree_43(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 18) {
      if (features[1] <= 312500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 23) {
        if (features[1] <= 175000) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 160000) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 375000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 65000) {
      if (features[0] <= 148) {
        if (features[1] <= 8750) {
          if (features[0] <= 94) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 18750) {
            if (features[0] <= 67) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 175) {
          if (features[1] <= 37500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 202) {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 37500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[1] <= 75000) {
          if (features[0] <= 121) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 85000) {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 108) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 187500) {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 44
int predict_tree_44(int features[]) {
  if (features[1] <= 47500) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 94) {
        if (features[1] <= 8750) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 1; // FDS_TCLOCK
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[1] <= 312500) {
        if (features[1] <= 187500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 15) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 562500) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 15) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[0] <= 121) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 80000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 187500) {
          if (features[1] <= 112500) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[0] <= 67) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 45
int predict_tree_45(int features[]) {
  if (features[1] <= 187500) {
    if (features[0] <= 40) {
      if (features[0] <= 23) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 102500) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 121) {
        if (features[1] <= 8750) {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 94) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 97500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 47500) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[1] <= 55000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 18) {
      if (features[0] <= 7) {
        return 0; // FDS_QSPINLOCK
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 46
int predict_tree_46(int features[]) {
  if (features[1] <= 85000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 47500) {
        if (features[1] <= 6250) {
          if (features[0] <= 162) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 11250) {
            if (features[1] <= 8750) {
              if (features[0] <= 108) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 148) {
          if (features[1] <= 75000) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 81) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 312500) {
      if (features[1] <= 112500) {
        if (features[0] <= 48) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 95000) {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 67) {
          if (features[1] <= 187500) {
            if (features[0] <= 15) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 12) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[1] <= 437500) {
          if (features[0] <= 32) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 15) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 12) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 47
int predict_tree_47(int features[]) {
  if (features[0] <= 67) {
    if (features[0] <= 23) {
      if (features[0] <= 13) {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 250000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 18) {
          if (features[1] <= 300000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 352500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 170000) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 23750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 187500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[1] <= 47500) {
        if (features[0] <= 148) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 162) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 94) {
        if (features[1] <= 95000) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 48
int predict_tree_48(int features[]) {
  if (features[1] <= 47500) {
    if (features[1] <= 18750) {
      if (features[1] <= 6250) {
        if (features[0] <= 148) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 67) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 40000) {
        if (features[1] <= 30000) {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 67) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 29) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[0] <= 23) {
        if (features[1] <= 312500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 562500) {
            if (features[1] <= 437500) {
              if (features[0] <= 7) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[0] <= 15) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            } else {
              if (features[0] <= 6) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[0] <= 18) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          } else {
            if (features[0] <= 12) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 97500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 437500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 75000) {
        if (features[0] <= 148) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 94) {
          if (features[1] <= 250000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 49
int predict_tree_49(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 312500) {
        if (features[0] <= 14) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 562500) {
            if (features[1] <= 437500) {
              if (features[0] <= 15) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 15) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 148) {
      if (features[0] <= 94) {
        if (features[1] <= 170000) {
          if (features[1] <= 11250) {
            if (features[0] <= 67) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            if (features[1] <= 18750) {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 75000) {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 47500) {
        return 1; // FDS_TCLOCK
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 50
int predict_tree_50(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 6) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 18) {
        if (features[0] <= 9) {
          if (features[1] <= 300000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 13) {
            if (features[1] <= 312500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 300000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[0] <= 23) {
          if (features[1] <= 300000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 107500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 250000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 121) {
      if (features[0] <= 67) {
        if (features[1] <= 187500) {
          if (features[1] <= 17500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 94) {
          if (features[1] <= 95000) {
            if (features[1] <= 8750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 97500) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 202) {
        if (features[1] <= 65000) {
          if (features[1] <= 6250) {
            if (features[0] <= 162) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 42500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 51
int predict_tree_51(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 562500) {
          if (features[1] <= 437500) {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 21) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 175) {
      if (features[1] <= 75000) {
        if (features[1] <= 11250) {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 8750) {
              if (features[0] <= 108) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 81) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 94) {
          if (features[1] <= 187500) {
            if (features[1] <= 95000) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[0] <= 67) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 202) {
        if (features[1] <= 52500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 36250) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 52
int predict_tree_52(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 7) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 15) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 23) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[1] <= 437500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 85000) {
      if (features[0] <= 121) {
        if (features[0] <= 67) {
          if (features[1] <= 18750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 8750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 175) {
          if (features[1] <= 70000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 37500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 202) {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 52500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 67) {
        return 1; // FDS_TCLOCK
      } else {
        if (features[1] <= 95000) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 53
int predict_tree_53(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 18750) {
        if (features[0] <= 67) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 148) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 94) {
                if (features[1] <= 8750) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 175) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 202) {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 187500) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 187500) {
        if (features[0] <= 67) {
          if (features[0] <= 40) {
            if (features[1] <= 102500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 40) {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 54
int predict_tree_54(int features[]) {
  if (features[1] <= 112500) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 121) {
        if (features[1] <= 11250) {
          if (features[0] <= 94) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 67) {
            if (features[1] <= 23750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 75000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 202) {
          if (features[0] <= 148) {
            if (features[1] <= 65000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 37500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 37500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 312500) {
      if (features[0] <= 40) {
        if (features[0] <= 23) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 18) {
        if (features[1] <= 437500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 4) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 55
int predict_tree_55(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 23) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 312500) {
        if (features[0] <= 12) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 437500) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 15) {
            if (features[1] <= 562500) {
              if (features[0] <= 6) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 7) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 55000) {
      if (features[0] <= 94) {
        if (features[1] <= 18750) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 6250) {
          if (features[0] <= 148) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 175) {
              if (features[1] <= 37500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 85000) {
        if (features[0] <= 121) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 187500) {
          if (features[1] <= 112500) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[0] <= 67) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 56
int predict_tree_56(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 112500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 312500) {
        if (features[0] <= 23) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 437500) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[0] <= 18) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[0] <= 175) {
        if (features[0] <= 121) {
          if (features[1] <= 8750) {
            if (features[0] <= 94) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 6250) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            if (features[1] <= 18750) {
              if (features[1] <= 11250) {
                return 1; // FDS_TCLOCK
              } else {
                if (features[0] <= 67) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 202) {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 37500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 187500) {
        if (features[1] <= 112500) {
          if (features[0] <= 94) {
            if (features[1] <= 95000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 57
int predict_tree_57(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 312500) {
      if (features[0] <= 23) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 107500) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 7) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 437500) {
          if (features[0] <= 15) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 562500) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 85000) {
      if (features[0] <= 148) {
        if (features[1] <= 6250) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 75000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 37500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 94) {
        if (features[1] <= 95000) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 67) {
            if (features[1] <= 250000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 58
int predict_tree_58(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 6) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 9) {
        if (features[1] <= 300000) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 23) {
          if (features[0] <= 13) {
            if (features[1] <= 375000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 227500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 107500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 121) {
      if (features[0] <= 67) {
        if (features[1] <= 28750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 187500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 94) {
          if (features[1] <= 107500) {
            if (features[1] <= 7500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 222500) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 202) {
        if (features[1] <= 70000) {
          if (features[1] <= 47500) {
            if (features[1] <= 6250) {
              if (features[0] <= 148) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 55000) {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 52500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 59
int predict_tree_59(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 312500) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 187500) {
          if (features[0] <= 23) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 12) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 15) {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  } else {
    if (features[0] <= 94) {
      if (features[0] <= 67) {
        if (features[1] <= 23750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 170000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 217500) {
          if (features[1] <= 8750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 202) {
        if (features[0] <= 175) {
          if (features[1] <= 75000) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 37500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 60
int predict_tree_60(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 312500) {
      if (features[0] <= 23) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 102500) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 18) {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[1] <= 6250) {
        if (features[0] <= 162) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 67) {
          if (features[1] <= 17500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 47500) {
            if (features[0] <= 94) {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[1] <= 85000) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 67) {
          if (features[1] <= 250000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 61
int predict_tree_61(int features[]) {
  if (features[1] <= 187500) {
    if (features[0] <= 40) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 23) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 94) {
        if (features[1] <= 11250) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 67) {
            if (features[1] <= 18750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 102500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 47500) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 148) {
            if (features[0] <= 121) {
              return 2; // FDS_TDLOCK
            } else {
              if (features[1] <= 87500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 312500) {
      if (features[0] <= 37) {
        if (features[0] <= 12) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[1] <= 437500) {
        if (features[0] <= 18) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 562500) {
          if (features[0] <= 14) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 15) {
            if (features[0] <= 7) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 62
int predict_tree_62(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 6) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 187500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 23) {
          if (features[1] <= 437500) {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 312500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[0] <= 67) {
        if (features[1] <= 18750) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 8750) {
          if (features[0] <= 148) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 94) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 162) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[1] <= 85000) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 108) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 67) {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 63
int predict_tree_63(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 8750) {
      if (features[0] <= 148) {
        if (features[0] <= 94) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 121) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 0; // FDS_QSPINLOCK
          }
        }
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 112500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 23) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 75000) {
          if (features[0] <= 162) {
            if (features[0] <= 67) {
              if (features[1] <= 23750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 94) {
            if (features[0] <= 67) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 102500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 562500) {
      if (features[0] <= 18) {
        if (features[1] <= 312500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 437500) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 7) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[1] <= 312500) {
          if (features[0] <= 40) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 15) {
        return 1; // FDS_TCLOCK
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 64
int predict_tree_64(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 112500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 18) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[1] <= 312500) {
            if (features[1] <= 187500) {
              if (features[0] <= 23) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 55000) {
      if (features[0] <= 67) {
        if (features[1] <= 23750) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 6250) {
          if (features[0] <= 148) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 47500) {
            if (features[0] <= 94) {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 175) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[1] <= 75000) {
          if (features[0] <= 135) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 85000) {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 65
int predict_tree_65(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 47500) {
        if (features[0] <= 148) {
          if (features[0] <= 67) {
            if (features[1] <= 21250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 8750) {
              if (features[1] <= 6250) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[0] <= 108) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 65000) {
          if (features[0] <= 135) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 85000) {
          if (features[0] <= 108) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[1] <= 437500) {
          if (features[1] <= 312500) {
            if (features[0] <= 40) {
              if (features[0] <= 12) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 15) {
              if (features[0] <= 7) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 18) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 29) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 66
int predict_tree_66(int features[]) {
  if (features[1] <= 112500) {
    if (features[1] <= 95000) {
      if (features[1] <= 55000) {
        if (features[0] <= 94) {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 67) {
              if (features[1] <= 22500) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          }
        } else {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 75000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 85000) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[0] <= 81) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      if (features[0] <= 50) {
        return 0; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  } else {
    if (features[0] <= 18) {
      if (features[1] <= 312500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 562500) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 0; // FDS_QSPINLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[0] <= 23) {
          return 2; // FDS_TDLOCK
        } else {
          if (features[1] <= 250000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 67
int predict_tree_67(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 9) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 23) {
        if (features[0] <= 18) {
          if (features[1] <= 250000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 187500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 107500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 94) {
      if (features[1] <= 18750) {
        if (features[1] <= 11250) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 67) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 95000) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 67) {
            if (features[1] <= 250000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 202) {
        if (features[1] <= 75000) {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 175) {
              if (features[1] <= 37500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 41250) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 68
int predict_tree_68(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 112500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 312500) {
        if (features[1] <= 187500) {
          if (features[0] <= 18) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 12) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 437500) {
          if (features[0] <= 15) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 562500) {
              if (features[0] <= 21) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 15) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 94) {
      if (features[0] <= 67) {
        if (features[1] <= 23750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 250000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 95000) {
          if (features[1] <= 8750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 47500) {
        if (features[1] <= 6250) {
          if (features[0] <= 148) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 148) {
          if (features[1] <= 75000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 69
int predict_tree_69(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 7) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 18) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 23) {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 94) {
      if (features[1] <= 11250) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 95000) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[1] <= 187500) {
            if (features[1] <= 112500) {
              return 2; // FDS_TDLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 121) {
        if (features[1] <= 80000) {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 202) {
          if (features[0] <= 148) {
            if (features[1] <= 70000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 37500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 31250) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 70
int predict_tree_70(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 18) {
      if (features[1] <= 437500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 23) {
          if (features[1] <= 312500) {
            if (features[1] <= 187500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 375000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 202) {
      if (features[1] <= 75000) {
        if (features[1] <= 6250) {
          if (features[0] <= 148) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 47500) {
            if (features[0] <= 81) {
              if (features[1] <= 18750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 55000) {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 162) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      } else {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[1] <= 90000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 37500) {
        return 1; // FDS_TCLOCK
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 71
int predict_tree_71(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 8750) {
        if (features[1] <= 6250) {
          if (features[0] <= 148) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 94) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 175) {
          if (features[0] <= 67) {
            if (features[1] <= 28750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 202) {
            if (features[1] <= 57500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 37500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[0] <= 9) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 18) {
          if (features[0] <= 13) {
            if (features[1] <= 250000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 0; // FDS_QSPINLOCK
          }
        } else {
          if (features[1] <= 187500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 23) {
              return 2; // FDS_TDLOCK
            } else {
              if (features[1] <= 375000) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[1] <= 85000) {
          if (features[0] <= 81) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 72
int predict_tree_72(int features[]) {
  if (features[1] <= 187500) {
    if (features[0] <= 40) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 23) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 121) {
        if (features[1] <= 8750) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 107500) {
            if (features[0] <= 67) {
              if (features[1] <= 18750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 94) {
                return 1; // FDS_TCLOCK
              } else {
                if (features[1] <= 75000) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 148) {
          if (features[1] <= 70000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 175) {
            if (features[1] <= 30000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 52500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 562500) {
      if (features[0] <= 18) {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 73
int predict_tree_73(int features[]) {
  if (features[1] <= 187500) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 75000) {
        if (features[1] <= 6250) {
          if (features[0] <= 175) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 202) {
            if (features[1] <= 18750) {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[0] <= 94) {
                  if (features[1] <= 8750) {
                    return 0; // FDS_QSPINLOCK
                  } else {
                    return 1; // FDS_TCLOCK
                  }
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            } else {
              if (features[0] <= 148) {
                return 1; // FDS_TCLOCK
              } else {
                if (features[1] <= 37500) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          } else {
            if (features[1] <= 31250) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 95000) {
          if (features[1] <= 85000) {
            if (features[0] <= 81) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 135) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[1] <= 437500) {
        if (features[1] <= 312500) {
          if (features[0] <= 11) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 12) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 562500) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 74
int predict_tree_74(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 55000) {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 8750) {
          if (features[0] <= 135) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 148) {
            if (features[1] <= 11250) {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 175) {
              if (features[1] <= 37500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 202) {
                return 1; // FDS_TCLOCK
              } else {
                if (features[1] <= 37500) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          }
        }
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 95000) {
          if (features[0] <= 121) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 112500) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[0] <= 121) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 437500) {
      if (features[1] <= 312500) {
        if (features[0] <= 40) {
          if (features[0] <= 12) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 28) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 562500) {
        if (features[0] <= 15) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 45) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 75
int predict_tree_75(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      if (features[1] <= 112500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 23) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 437500) {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 562500) {
            if (features[0] <= 21) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 18) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 55000) {
      if (features[0] <= 148) {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            if (features[1] <= 18750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 8750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 7500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 47500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[0] <= 94) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[1] <= 75000) {
            if (features[0] <= 162) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 76
int predict_tree_76(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 65000) {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 18750) {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 8750) {
              if (features[0] <= 94) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          }
        } else {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 85000) {
          if (features[1] <= 75000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 18) {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 40) {
        if (features[0] <= 23) {
          return 2; // FDS_TDLOCK
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 77
int predict_tree_77(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 47500) {
      if (features[1] <= 11250) {
        if (features[0] <= 94) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 54) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 85000) {
          if (features[1] <= 55000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 65000) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 75000) {
                if (features[0] <= 148) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                if (features[0] <= 94) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          }
        } else {
          if (features[1] <= 112500) {
            if (features[1] <= 95000) {
              if (features[0] <= 94) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 67) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 312500) {
      if (features[0] <= 37) {
        if (features[0] <= 12) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 14) {
        if (features[1] <= 437500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 562500) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 0; // FDS_QSPINLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 78
int predict_tree_78(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 47500) {
        if (features[0] <= 67) {
          if (features[1] <= 28750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 148) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[1] <= 85000) {
        if (features[0] <= 67) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 95000) {
          if (features[0] <= 54) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 94) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 112500) {
            if (features[0] <= 77) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 23) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 67) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      if (features[1] <= 312500) {
        if (features[0] <= 40) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 15) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 79
int predict_tree_79(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 312500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 562500) {
        if (features[0] <= 15) {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[1] <= 8750) {
        if (features[0] <= 148) {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 94) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 47500) {
          if (features[1] <= 18750) {
            if (features[1] <= 11250) {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 67) {
        if (features[1] <= 187500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 80
int predict_tree_80(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 6) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 187500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 18) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 23) {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 65000) {
      if (features[0] <= 94) {
        if (features[0] <= 67) {
          if (features[1] <= 17500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 8750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 121) {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 148) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 94) {
        if (features[0] <= 67) {
          if (features[1] <= 170000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 85000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 81
int predict_tree_81(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 9) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 18) {
        if (features[1] <= 250000) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 23) {
          if (features[1] <= 312500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 170000) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 148) {
      if (features[1] <= 75000) {
        if (features[0] <= 67) {
          if (features[1] <= 18750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 94) {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[0] <= 94) {
          if (features[1] <= 90000) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 67) {
              if (features[1] <= 250000) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 202) {
        if (features[0] <= 175) {
          if (features[1] <= 30000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 42500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 82
int predict_tree_82(int features[]) {
  if (features[1] <= 187500) {
    if (features[1] <= 30000) {
      if (features[1] <= 6250) {
        if (features[0] <= 148) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 67) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 112500) {
          if (features[0] <= 48) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 23) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 54) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 562500) {
      if (features[1] <= 312500) {
        if (features[0] <= 67) {
          if (features[0] <= 12) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 437500) {
          if (features[0] <= 15) {
            if (features[0] <= 7) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 18) {
            if (features[0] <= 10) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 18) {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 83
int predict_tree_83(int features[]) {
  if (features[1] <= 312500) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 148) {
        if (features[0] <= 94) {
          if (features[1] <= 11250) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 187500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 75000) {
            if (features[1] <= 6250) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 202) {
          if (features[0] <= 175) {
            if (features[1] <= 37500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 62500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 437500) {
      if (features[0] <= 15) {
        return 0; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 18) {
        if (features[1] <= 562500) {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 84
int predict_tree_84(int features[]) {
  if (features[1] <= 75000) {
    if (features[1] <= 47500) {
      if (features[1] <= 6250) {
        if (features[0] <= 162) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 18750) {
            if (features[1] <= 11250) {
              if (features[0] <= 94) {
                if (features[0] <= 67) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  if (features[1] <= 8750) {
                    return 0; // FDS_QSPINLOCK
                  } else {
                    return 1; // FDS_TCLOCK
                  }
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 67) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 81) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 148) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 162) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 9) {
          if (features[1] <= 300000) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 18) {
            if (features[1] <= 250000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 175000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 187500) {
        if (features[1] <= 112500) {
          if (features[0] <= 94) {
            if (features[0] <= 54) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 67) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 85
int predict_tree_85(int features[]) {
  if (features[1] <= 95000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 47500) {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            if (features[1] <= 15000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 10000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 148) {
            if (features[0] <= 121) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 15000) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 148) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 75000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 562500) {
      if (features[1] <= 437500) {
        if (features[0] <= 67) {
          if (features[0] <= 15) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 312500) {
              if (features[0] <= 23) {
                if (features[1] <= 187500) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 18) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 86
int predict_tree_86(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 562500) {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 437500) {
              if (features[0] <= 18) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 18) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[0] <= 15) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 94) {
      if (features[1] <= 85000) {
        if (features[1] <= 18750) {
          if (features[0] <= 67) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 10000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 148) {
        if (features[1] <= 75000) {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 175) {
          if (features[1] <= 28750) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 87
int predict_tree_87(int features[]) {
  if (features[1] <= 85000) {
    if (features[1] <= 11250) {
      if (features[0] <= 148) {
        if (features[0] <= 67) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 121) {
            if (features[0] <= 94) {
              if (features[1] <= 8750) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 6250) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 0; // FDS_QSPINLOCK
          }
        }
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 202) {
          if (features[0] <= 148) {
            if (features[1] <= 75000) {
              if (features[0] <= 67) {
                if (features[1] <= 18750) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 81) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            if (features[1] <= 37500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 562500) {
      if (features[1] <= 437500) {
        if (features[0] <= 18) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 95000) {
            if (features[0] <= 94) {
              if (features[0] <= 40) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 312500) {
              if (features[0] <= 40) {
                if (features[1] <= 112500) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                if (features[0] <= 67) {
                  if (features[1] <= 187500) {
                    return 1; // FDS_TCLOCK
                  } else {
                    return 2; // FDS_TDLOCK
                  }
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 35) {
          if (features[0] <= 4) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 88
int predict_tree_88(int features[]) {
  if (features[1] <= 85000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 55000) {
        if (features[1] <= 18750) {
          if (features[1] <= 11250) {
            if (features[1] <= 6250) {
              if (features[0] <= 148) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 94) {
                if (features[0] <= 67) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  if (features[1] <= 8750) {
                    return 0; // FDS_QSPINLOCK
                  } else {
                    return 1; // FDS_TCLOCK
                  }
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            if (features[0] <= 121) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 94) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 148) {
            if (features[1] <= 70000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 187500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 18) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 375000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 67) {
        if (features[1] <= 170000) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 40) {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 89
int predict_tree_89(int features[]) {
  if (features[1] <= 85000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 202) {
        if (features[0] <= 121) {
          if (features[1] <= 8750) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 148) {
            if (features[1] <= 62500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 57500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 31250) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[1] <= 437500) {
        if (features[0] <= 9) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 187500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 562500) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[0] <= 12) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[0] <= 94) {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 312500) {
          if (features[1] <= 112500) {
            return 2; // FDS_TDLOCK
          } else {
            if (features[0] <= 81) {
              if (features[1] <= 187500) {
                return 1; // FDS_TCLOCK
              } else {
                if (features[0] <= 40) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 90
int predict_tree_90(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 18) {
      if (features[1] <= 375000) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 562500) {
          if (features[0] <= 5) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 7) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 23) {
        if (features[1] <= 227500) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 107500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 312500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 95000) {
      if (features[1] <= 47500) {
        if (features[1] <= 11250) {
          if (features[1] <= 8750) {
            if (features[0] <= 148) {
              if (features[0] <= 94) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[1] <= 6250) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              }
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 81) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 148) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 121) {
              if (features[1] <= 75000) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[1] <= 70000) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 91
int predict_tree_91(int features[]) {
  if (features[1] <= 95000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 175) {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            if (features[1] <= 28750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 8750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 75000) {
            if (features[0] <= 148) {
              if (features[1] <= 6250) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 30000) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 37500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 187500) {
      if (features[0] <= 21) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 112500) {
          return 2; // FDS_TDLOCK
        } else {
          if (features[0] <= 54) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 18) {
        if (features[1] <= 312500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 312500) {
          if (features[0] <= 81) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 92
int predict_tree_92(int features[]) {
  if (features[1] <= 85000) {
    if (features[0] <= 67) {
      if (features[1] <= 75000) {
        if (features[1] <= 30000) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 40) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 202) {
          if (features[0] <= 94) {
            if (features[1] <= 8750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 30000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 148) {
          if (features[0] <= 94) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 75000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[1] <= 187500) {
        if (features[0] <= 23) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 562500) {
          if (features[1] <= 437500) {
            if (features[1] <= 312500) {
              if (features[0] <= 12) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 6) {
                return 0; // FDS_QSPINLOCK
              } else {
                if (features[0] <= 15) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          } else {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 94) {
        if (features[1] <= 187500) {
          if (features[1] <= 107500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 67) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 93
int predict_tree_93(int features[]) {
  if (features[1] <= 75000) {
    if (features[1] <= 8750) {
      if (features[1] <= 6250) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 94) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 40000) {
        if (features[1] <= 18750) {
          if (features[1] <= 11250) {
            if (features[0] <= 67) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 67) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 30000) {
            if (features[0] <= 40) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 40) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[0] <= 40) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 47500) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[0] <= 162) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 562500) {
      if (features[1] <= 437500) {
        if (features[0] <= 40) {
          if (features[1] <= 187500) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 15) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 312500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[1] <= 187500) {
            if (features[0] <= 81) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 35) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 12) {
        return 0; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 94
int predict_tree_94(int features[]) {
  if (features[1] <= 112500) {
    if (features[1] <= 8750) {
      if (features[0] <= 162) {
        if (features[0] <= 94) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 6250) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        return 1; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 40) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 175) {
          if (features[0] <= 148) {
            if (features[0] <= 67) {
              if (features[1] <= 17500) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 94) {
                if (features[1] <= 95000) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                if (features[1] <= 75000) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          } else {
            if (features[1] <= 52500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 202) {
            if (features[1] <= 47500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 30000) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 312500) {
      if (features[0] <= 12) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 187500) {
          if (features[0] <= 81) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 40) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 18) {
        if (features[1] <= 437500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 562500) {
            if (features[0] <= 6) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 7) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 95
int predict_tree_95(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 23) {
      if (features[1] <= 187500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 375000) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[1] <= 562500) {
            if (features[0] <= 5) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 0; // FDS_QSPINLOCK
          }
        }
      }
    } else {
      if (features[1] <= 107500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 375000) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 65000) {
      if (features[1] <= 6250) {
        if (features[0] <= 175) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 47500) {
          if (features[1] <= 18750) {
            if (features[1] <= 11250) {
              if (features[0] <= 94) {
                if (features[1] <= 8750) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  if (features[0] <= 67) {
                    return 0; // FDS_QSPINLOCK
                  } else {
                    return 1; // FDS_TCLOCK
                  }
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 81) {
                return 0; // FDS_QSPINLOCK
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 135) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 94) {
        if (features[1] <= 95000) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[1] <= 187500) {
            if (features[0] <= 67) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 96
int predict_tree_96(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 187500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 437500) {
        if (features[1] <= 312500) {
          if (features[0] <= 11) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 6) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 15) {
          if (features[1] <= 562500) {
            if (features[0] <= 5) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 7) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 175) {
      if (features[0] <= 67) {
        if (features[1] <= 250000) {
          if (features[1] <= 17500) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 94) {
          if (features[1] <= 95000) {
            if (features[1] <= 8750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 70000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 37500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 202) {
        if (features[1] <= 37500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 37500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 97
int predict_tree_97(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 23) {
      if (features[1] <= 437500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          return 0; // FDS_QSPINLOCK
        } else {
          return 1; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 107500) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 312500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 95000) {
      if (features[1] <= 18750) {
        if (features[1] <= 8750) {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 108) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 67) {
            return 0; // FDS_QSPINLOCK
          } else {
            return 1; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 47500) {
          return 1; // FDS_TCLOCK
        } else {
          if (features[1] <= 55000) {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 75000) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 85000) {
                if (features[0] <= 121) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              } else {
                if (features[0] <= 94) {
                  return 1; // FDS_TCLOCK
                } else {
                  return 2; // FDS_TDLOCK
                }
              }
            }
          }
        }
      }
    } else {
      return 2; // FDS_TDLOCK
    }
  }
}

// Tree 98
int predict_tree_98(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 40) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 175) {
        if (features[0] <= 94) {
          if (features[0] <= 67) {
            if (features[1] <= 23750) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 10000) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 6250) {
            if (features[0] <= 148) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 148) {
              return 1; // FDS_TCLOCK
            } else {
              if (features[1] <= 37500) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          }
        }
      } else {
        if (features[1] <= 37500) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 23) {
      if (features[0] <= 6) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 187500) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 15) {
            return 1; // FDS_TCLOCK
          } else {
            if (features[1] <= 312500) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 94) {
        if (features[0] <= 40) {
          if (features[1] <= 375000) {
            if (features[1] <= 107500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 95000) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        }
      } else {
        return 2; // FDS_TDLOCK
      }
    }
  }
}

// Tree 99
int predict_tree_99(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 112500) {
      return 0; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 312500) {
        if (features[0] <= 23) {
          if (features[0] <= 15) {
            return 0; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 187500) {
              return 0; // FDS_QSPINLOCK
            } else {
              return 1; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 7) {
          return 0; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 562500) {
            if (features[1] <= 437500) {
              if (features[0] <= 15) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 21) {
                return 1; // FDS_TCLOCK
              } else {
                return 2; // FDS_TDLOCK
              }
            }
          } else {
            if (features[0] <= 18) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 65000) {
      if (features[1] <= 6250) {
        return 0; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 47500) {
          if (features[0] <= 94) {
            if (features[1] <= 11250) {
              return 0; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 67) {
                if (features[1] <= 18750) {
                  return 0; // FDS_QSPINLOCK
                } else {
                  return 1; // FDS_TCLOCK
                }
              } else {
                return 1; // FDS_TCLOCK
              }
            }
          } else {
            return 1; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 55000) {
            if (features[0] <= 135) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 162) {
              return 1; // FDS_TCLOCK
            } else {
              return 2; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 95000) {
        if (features[0] <= 94) {
          return 1; // FDS_TCLOCK
        } else {
          return 2; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 67) {
          if (features[1] <= 187500) {
            return 1; // FDS_TCLOCK
          } else {
            return 2; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TDLOCK
        }
      }
    }
  }
}

int predict_random_forest(int features[]) {
  int predictions[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  predictions[0] = predict_tree_0(features);
  predictions[1] = predict_tree_1(features);
  predictions[2] = predict_tree_2(features);
  predictions[3] = predict_tree_3(features);
  predictions[4] = predict_tree_4(features);
  predictions[5] = predict_tree_5(features);
  predictions[6] = predict_tree_6(features);
  predictions[7] = predict_tree_7(features);
  predictions[8] = predict_tree_8(features);
  predictions[9] = predict_tree_9(features);
  predictions[10] = predict_tree_10(features);
  predictions[11] = predict_tree_11(features);
  predictions[12] = predict_tree_12(features);
  predictions[13] = predict_tree_13(features);
  predictions[14] = predict_tree_14(features);
  predictions[15] = predict_tree_15(features);
  predictions[16] = predict_tree_16(features);
  predictions[17] = predict_tree_17(features);
  predictions[18] = predict_tree_18(features);
  predictions[19] = predict_tree_19(features);
  predictions[20] = predict_tree_20(features);
  predictions[21] = predict_tree_21(features);
  predictions[22] = predict_tree_22(features);
  predictions[23] = predict_tree_23(features);
  predictions[24] = predict_tree_24(features);
  predictions[25] = predict_tree_25(features);
  predictions[26] = predict_tree_26(features);
  predictions[27] = predict_tree_27(features);
  predictions[28] = predict_tree_28(features);
  predictions[29] = predict_tree_29(features);
  predictions[30] = predict_tree_30(features);
  predictions[31] = predict_tree_31(features);
  predictions[32] = predict_tree_32(features);
  predictions[33] = predict_tree_33(features);
  predictions[34] = predict_tree_34(features);
  predictions[35] = predict_tree_35(features);
  predictions[36] = predict_tree_36(features);
  predictions[37] = predict_tree_37(features);
  predictions[38] = predict_tree_38(features);
  predictions[39] = predict_tree_39(features);
  predictions[40] = predict_tree_40(features);
  predictions[41] = predict_tree_41(features);
  predictions[42] = predict_tree_42(features);
  predictions[43] = predict_tree_43(features);
  predictions[44] = predict_tree_44(features);
  predictions[45] = predict_tree_45(features);
  predictions[46] = predict_tree_46(features);
  predictions[47] = predict_tree_47(features);
  predictions[48] = predict_tree_48(features);
  predictions[49] = predict_tree_49(features);
  predictions[50] = predict_tree_50(features);
  predictions[51] = predict_tree_51(features);
  predictions[52] = predict_tree_52(features);
  predictions[53] = predict_tree_53(features);
  predictions[54] = predict_tree_54(features);
  predictions[55] = predict_tree_55(features);
  predictions[56] = predict_tree_56(features);
  predictions[57] = predict_tree_57(features);
  predictions[58] = predict_tree_58(features);
  predictions[59] = predict_tree_59(features);
  predictions[60] = predict_tree_60(features);
  predictions[61] = predict_tree_61(features);
  predictions[62] = predict_tree_62(features);
  predictions[63] = predict_tree_63(features);
  predictions[64] = predict_tree_64(features);
  predictions[65] = predict_tree_65(features);
  predictions[66] = predict_tree_66(features);
  predictions[67] = predict_tree_67(features);
  predictions[68] = predict_tree_68(features);
  predictions[69] = predict_tree_69(features);
  predictions[70] = predict_tree_70(features);
  predictions[71] = predict_tree_71(features);
  predictions[72] = predict_tree_72(features);
  predictions[73] = predict_tree_73(features);
  predictions[74] = predict_tree_74(features);
  predictions[75] = predict_tree_75(features);
  predictions[76] = predict_tree_76(features);
  predictions[77] = predict_tree_77(features);
  predictions[78] = predict_tree_78(features);
  predictions[79] = predict_tree_79(features);
  predictions[80] = predict_tree_80(features);
  predictions[81] = predict_tree_81(features);
  predictions[82] = predict_tree_82(features);
  predictions[83] = predict_tree_83(features);
  predictions[84] = predict_tree_84(features);
  predictions[85] = predict_tree_85(features);
  predictions[86] = predict_tree_86(features);
  predictions[87] = predict_tree_87(features);
  predictions[88] = predict_tree_88(features);
  predictions[89] = predict_tree_89(features);
  predictions[90] = predict_tree_90(features);
  predictions[91] = predict_tree_91(features);
  predictions[92] = predict_tree_92(features);
  predictions[93] = predict_tree_93(features);
  predictions[94] = predict_tree_94(features);
  predictions[95] = predict_tree_95(features);
  predictions[96] = predict_tree_96(features);
  predictions[97] = predict_tree_97(features);
  predictions[98] = predict_tree_98(features);
  predictions[99] = predict_tree_99(features);
  int counts[3] = {0, 0, 0};
  for (int i = 0; i < 100; i++) {
    counts[predictions[i]]++;
  }
  int max_count = 0;
  int max_index = 0;
  for (int i = 0; i < 3; i++) {
    if (counts[i] > max_count) {
      max_count = counts[i];
      max_index = i;
    }
  }
  return max_index;
}

