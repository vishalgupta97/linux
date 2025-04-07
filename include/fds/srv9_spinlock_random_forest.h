/*
 feature_vector[0] - Core
feature_vector[1] - RPS

output_index[0] - FDS_AQS
output_index[1] - FDS_QSPINLOCK
output_index[2] - FDS_TCLOCK
output_index[3] - FDS_TDLOCK
*/
// Tree 0
int spinlock_predict_tree_0(int features[]) {
  if (features[1] <= 35000) {
    if (features[1] <= 7500) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 51) {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 12) {
      if (features[1] <= 150000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 6) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[1] <= 55000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 125000) {
            if (features[0] <= 28) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 1
int spinlock_predict_tree_1(int features[]) {
  if (features[1] <= 35000) {
    if (features[0] <= 51) {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[1] <= 7500) {
          if (features[0] <= 68) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 80) {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[0] <= 40) {
          return 3; // FDS_TDLOCK
        } else {
          if (features[1] <= 50000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 2
int spinlock_predict_tree_2(int features[]) {
  if (features[1] <= 35000) {
    if (features[1] <= 7500) {
      if (features[0] <= 80) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 25) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 25000) {
          if (features[0] <= 51) {
            if (features[1] <= 15000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 45) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 12) {
        if (features[1] <= 75000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 85000) {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_QSPINLOCK
          }
        }
      } else {
        if (features[1] <= 125000) {
          if (features[1] <= 45000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 55000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 3
int spinlock_predict_tree_3(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 125000) {
      if (features[1] <= 45000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 75000) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 12) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[0] <= 40) {
        if (features[1] <= 12500) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 25000) {
          if (features[1] <= 7500) {
            if (features[0] <= 74) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[1] <= 45000) {
          return 3; // FDS_TDLOCK
        } else {
          if (features[0] <= 45) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 4
int spinlock_predict_tree_4(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 45000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 6) {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 175000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 0; // FDS_AQS
            }
          }
        } else {
          if (features[1] <= 175000) {
            if (features[0] <= 12) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 86) {
      if (features[1] <= 25000) {
        if (features[1] <= 15000) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 57) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 40000) {
          if (features[0] <= 45) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 25000) {
        return 2; // FDS_TCLOCK
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 5
int spinlock_predict_tree_5(int features[]) {
  if (features[1] <= 35000) {
    if (features[0] <= 28) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 63) {
        if (features[0] <= 40) {
          if (features[1] <= 20000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 7500) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 25000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        }
      } else {
        if (features[1] <= 7500) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 86) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 25000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[0] <= 12) {
          if (features[1] <= 85000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 45000) {
          if (features[0] <= 51) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 27) {
        if (features[1] <= 175000) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          if (features[0] <= 6) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 6
int spinlock_predict_tree_6(int features[]) {
  if (features[1] <= 35000) {
    if (features[1] <= 7500) {
      if (features[0] <= 80) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 25000) {
        if (features[1] <= 15000) {
          if (features[0] <= 57) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 40) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 42) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 74) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[1] <= 125000) {
        if (features[1] <= 85000) {
          if (features[1] <= 45000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 65000) {
              if (features[0] <= 12) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 75000) {
                if (features[0] <= 15) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              } else {
                if (features[0] <= 6) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              }
            }
          }
        } else {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 175000) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          if (features[0] <= 5) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[0] <= 40) {
          return 3; // FDS_TDLOCK
        } else {
          if (features[1] <= 50000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 7
int spinlock_predict_tree_7(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 45000) {
      if (features[1] <= 15000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 25000) {
          if (features[0] <= 25) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      }
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 28) {
            if (features[0] <= 12) {
              if (features[1] <= 75000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 55000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 175000) {
            if (features[0] <= 12) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[0] <= 80) {
        if (features[1] <= 12500) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 45000) {
        if (features[0] <= 51) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 74) {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              if (features[1] <= 35000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 8
int spinlock_predict_tree_8(int features[]) {
  if (features[1] <= 25000) {
    if (features[0] <= 51) {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 40) {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      }
    } else {
      if (features[1] <= 7500) {
        if (features[0] <= 74) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 85000) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 95000) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 5) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          }
        }
      } else {
        if (features[0] <= 40) {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 51) {
            if (features[1] <= 55000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 21) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 6) {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 9
int spinlock_predict_tree_9(int features[]) {
  if (features[1] <= 55000) {
    if (features[0] <= 28) {
      if (features[0] <= 12) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 19) {
          if (features[1] <= 45000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 40000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[1] <= 25000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 74) {
          if (features[0] <= 63) {
            if (features[1] <= 25000) {
              if (features[1] <= 7500) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 25000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 86) {
            if (features[1] <= 20000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 30000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 95000) {
      if (features[1] <= 85000) {
        if (features[1] <= 75000) {
          if (features[0] <= 28) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 28) {
            if (features[0] <= 5) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 28) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 46) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 12) {
          if (features[1] <= 175000) {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 0; // FDS_AQS
            }
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 10
int spinlock_predict_tree_10(int features[]) {
  if (features[1] <= 35000) {
    if (features[1] <= 7500) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 45) {
        if (features[0] <= 28) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 74) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[0] <= 12) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 45000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 51) {
          if (features[0] <= 40) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[1] <= 55000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 6) {
        if (features[0] <= 2) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      } else {
        if (features[0] <= 12) {
          if (features[1] <= 175000) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 11
int spinlock_predict_tree_11(int features[]) {
  if (features[1] <= 35000) {
    if (features[1] <= 25000) {
      if (features[1] <= 15000) {
        if (features[1] <= 7500) {
          if (features[0] <= 74) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 57) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 28) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 12) {
        if (features[1] <= 75000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 95000) {
            if (features[1] <= 85000) {
              if (features[0] <= 5) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 2) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 175000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 0; // FDS_AQS
              }
            }
          }
        }
      } else {
        if (features[0] <= 19) {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[1] <= 55000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 12
int spinlock_predict_tree_12(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 175000) {
      if (features[1] <= 55000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 12) {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 75000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 4) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      }
    } else {
      if (features[0] <= 5) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[1] <= 7500) {
        if (features[0] <= 63) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 15000) {
          if (features[0] <= 63) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[0] <= 74) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[1] <= 55000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 13
int spinlock_predict_tree_13(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 75000) {
      if (features[1] <= 55000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 12) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 175000) {
        if (features[1] <= 85000) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 10) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 6) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[1] <= 15000) {
        if (features[0] <= 74) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 80) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[1] <= 45000) {
          if (features[0] <= 51) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 45) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 14
int spinlock_predict_tree_14(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 6) {
      if (features[1] <= 175000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 2) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[1] <= 45000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          if (features[1] <= 75000) {
            if (features[1] <= 65000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[0] <= 12) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 15000) {
        if (features[0] <= 74) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 51) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 51) {
        if (features[0] <= 40) {
          return 3; // FDS_TDLOCK
        } else {
          if (features[1] <= 50000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 15
int spinlock_predict_tree_15(int features[]) {
  if (features[0] <= 12) {
    if (features[1] <= 150000) {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 65000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 2) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 0; // FDS_AQS
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 15000) {
        if (features[0] <= 80) {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 7500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 40) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 28) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 51) {
            if (features[1] <= 60000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 16
int spinlock_predict_tree_16(int features[]) {
  if (features[1] <= 35000) {
    if (features[0] <= 51) {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 20000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 63) {
        if (features[1] <= 12500) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 86) {
          if (features[1] <= 7500) {
            if (features[0] <= 74) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[1] <= 55000) {
        if (features[0] <= 36) {
          if (features[0] <= 12) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 45000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 85000) {
          if (features[1] <= 75000) {
            if (features[0] <= 28) {
              if (features[0] <= 12) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 25) {
              if (features[0] <= 6) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 28) {
            if (features[1] <= 95000) {
              if (features[0] <= 10) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 6) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 21) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 13) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 17
int spinlock_predict_tree_17(int features[]) {
  if (features[0] <= 6) {
    return 1; // FDS_QSPINLOCK
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 12) {
        if (features[1] <= 125000) {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 0; // FDS_AQS
        }
      } else {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 19) {
            if (features[1] <= 120000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 150000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[1] <= 7500) {
          if (features[0] <= 74) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 15000) {
            if (features[0] <= 57) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 51) {
          if (features[0] <= 40) {
            if (features[1] <= 60000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 55000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 18
int spinlock_predict_tree_18(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 12) {
      if (features[1] <= 95000) {
        if (features[1] <= 75000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 85000) {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_QSPINLOCK
          }
        }
      } else {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[0] <= 6) {
              if (features[1] <= 175000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 0; // FDS_AQS
              }
            } else {
              return 0; // FDS_AQS
            }
          }
        }
      }
    } else {
      if (features[1] <= 50000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 19) {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[0] <= 51) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 63) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 55000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 74) {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[1] <= 35000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 19
int spinlock_predict_tree_19(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 6) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 19) {
        if (features[0] <= 12) {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 125000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[1] <= 175000) {
                return 0; // FDS_AQS
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[1] <= 55000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 125000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[1] <= 55000) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 63) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 74) {
            if (features[1] <= 7500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 20
int spinlock_predict_tree_20(int features[]) {
  if (features[1] <= 25000) {
    if (features[1] <= 15000) {
      if (features[1] <= 7500) {
        if (features[0] <= 80) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 51) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[0] <= 12) {
      if (features[1] <= 125000) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 175000) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      }
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 28) {
          if (features[0] <= 19) {
            if (features[1] <= 50000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 35000) {
            if (features[0] <= 74) {
              if (features[0] <= 45) {
                return 2; // FDS_TCLOCK
              } else {
                if (features[0] <= 63) {
                  return 3; // FDS_TDLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 55000) {
              if (features[0] <= 51) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 21
int spinlock_predict_tree_21(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 55000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 175000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[1] <= 125000) {
          if (features[1] <= 70000) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 175000) {
            if (features[0] <= 12) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 15000) {
        if (features[1] <= 7500) {
          if (features[0] <= 74) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 35000) {
        if (features[0] <= 74) {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 22
int spinlock_predict_tree_22(int features[]) {
  if (features[0] <= 19) {
    if (features[1] <= 95000) {
      if (features[1] <= 45000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 12) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 6) {
          if (features[1] <= 175000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 2) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 0; // FDS_AQS
            }
          }
        } else {
          return 0; // FDS_AQS
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 7500) {
          if (features[0] <= 80) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 15000) {
            if (features[0] <= 57) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[1] <= 35000) {
        if (features[0] <= 74) {
          if (features[0] <= 45) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 40) {
          if (features[1] <= 65000) {
            if (features[0] <= 28) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[1] <= 55000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            if (features[1] <= 150000) {
              if (features[0] <= 28) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 23
int spinlock_predict_tree_23(int features[]) {
  if (features[0] <= 12) {
    if (features[0] <= 3) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      } else {
        if (features[1] <= 85000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[0] <= 51) {
        if (features[0] <= 28) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 40) {
            if (features[1] <= 15000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_QSPINLOCK
          }
        }
      } else {
        if (features[1] <= 25000) {
          if (features[0] <= 63) {
            if (features[1] <= 12500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 28) {
        if (features[1] <= 150000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 51) {
          if (features[1] <= 50000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 24
int spinlock_predict_tree_24(int features[]) {
  if (features[0] <= 12) {
    if (features[1] <= 75000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 115000) {
          return 2; // FDS_TCLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 7500) {
          if (features[0] <= 74) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 28) {
        if (features[1] <= 125000) {
          if (features[0] <= 19) {
            if (features[1] <= 50000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 55000) {
          if (features[1] <= 45000) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[0] <= 51) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 25
int spinlock_predict_tree_25(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 45000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[0] <= 12) {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 125000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[1] <= 175000) {
                return 0; // FDS_AQS
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[1] <= 25000) {
        if (features[0] <= 51) {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 63) {
          return 3; // FDS_TDLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      return 3; // FDS_TDLOCK
    }
  }
}

// Tree 26
int spinlock_predict_tree_26(int features[]) {
  if (features[1] <= 45000) {
    if (features[1] <= 15000) {
      if (features[1] <= 7500) {
        if (features[0] <= 68) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 57) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 19) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 35000) {
          if (features[0] <= 28) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 45) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[0] <= 63) {
                return 3; // FDS_TDLOCK
              } else {
                if (features[0] <= 74) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              }
            }
          }
        } else {
          if (features[0] <= 36) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 6) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 175000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 12) {
            if (features[1] <= 70000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 55000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 27
int spinlock_predict_tree_27(int features[]) {
  if (features[0] <= 12) {
    if (features[1] <= 175000) {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 80000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      }
    } else {
      return 3; // FDS_TDLOCK
    }
  } else {
    if (features[0] <= 40) {
      if (features[0] <= 19) {
        if (features[1] <= 55000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 120000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 35000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[0] <= 28) {
              if (features[1] <= 150000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 74) {
        if (features[0] <= 51) {
          if (features[1] <= 50000) {
            if (features[1] <= 25000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 63) {
            if (features[1] <= 27500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 45000) {
              if (features[1] <= 17500) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 28
int spinlock_predict_tree_28(int features[]) {
  if (features[1] <= 45000) {
    if (features[1] <= 7500) {
      if (features[0] <= 80) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 35000) {
        if (features[0] <= 28) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 74) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 25000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 36) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 6) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 125000) {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 65000) {
              if (features[0] <= 12) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 75000) {
                if (features[0] <= 25) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              } else {
                if (features[0] <= 28) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              }
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 29
int spinlock_predict_tree_29(int features[]) {
  if (features[0] <= 12) {
    if (features[0] <= 6) {
      if (features[1] <= 175000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 2) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[1] <= 80000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 145000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[1] <= 7500) {
        if (features[0] <= 74) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 28) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 25000) {
            if (features[0] <= 45) {
              if (features[1] <= 15000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 74) {
              if (features[0] <= 63) {
                if (features[0] <= 45) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 75000) {
        if (features[0] <= 28) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 51) {
            if (features[0] <= 40) {
              return 3; // FDS_TDLOCK
            } else {
              if (features[1] <= 50000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 25) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 30
int spinlock_predict_tree_30(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 12) {
      if (features[0] <= 6) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 175000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[1] <= 75000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 19) {
        if (features[1] <= 55000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 40000) {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 28) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 28) {
            if (features[1] <= 150000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 65000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[1] <= 7500) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 25000) {
          if (features[1] <= 15000) {
            if (features[0] <= 51) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 80) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 45000) {
        if (features[0] <= 51) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 31
int spinlock_predict_tree_31(int features[]) {
  if (features[1] <= 125000) {
    if (features[1] <= 25000) {
      if (features[0] <= 63) {
        if (features[1] <= 7500) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 15000) {
            if (features[0] <= 51) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 28) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[0] <= 74) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 85000) {
        if (features[0] <= 28) {
          if (features[1] <= 45000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 51) {
            if (features[0] <= 40) {
              return 3; // FDS_TDLOCK
            } else {
              if (features[1] <= 55000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 95000) {
          if (features[0] <= 4) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 40) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 13) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 28) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[0] <= 5) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 21) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 32
int spinlock_predict_tree_32(int features[]) {
  if (features[0] <= 12) {
    if (features[1] <= 175000) {
      if (features[1] <= 125000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[0] <= 4) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[1] <= 45000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 63) {
        if (features[1] <= 50000) {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 51) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[1] <= 30000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 33
int spinlock_predict_tree_33(int features[]) {
  if (features[0] <= 12) {
    if (features[1] <= 175000) {
      if (features[1] <= 85000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 95000) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      }
    } else {
      if (features[0] <= 3) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 51) {
      if (features[1] <= 25000) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 28) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 95000) {
          if (features[1] <= 75000) {
            if (features[1] <= 45000) {
              if (features[1] <= 35000) {
                if (features[0] <= 25) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              } else {
                if (features[0] <= 31) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              }
            } else {
              if (features[1] <= 65000) {
                return 2; // FDS_TCLOCK
              } else {
                if (features[0] <= 28) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              }
            }
          } else {
            if (features[1] <= 85000) {
              if (features[0] <= 31) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 34) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 63) {
        if (features[1] <= 25000) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 74) {
            if (features[1] <= 35000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 34
int spinlock_predict_tree_34(int features[]) {
  if (features[1] <= 25000) {
    if (features[0] <= 28) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 15000) {
        if (features[1] <= 7500) {
          if (features[0] <= 74) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 19) {
            if (features[0] <= 12) {
              if (features[1] <= 75000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 55000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            if (features[1] <= 40000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 175000) {
            if (features[0] <= 12) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[0] <= 40) {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 50000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 35
int spinlock_predict_tree_35(int features[]) {
  if (features[1] <= 35000) {
    if (features[0] <= 28) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 7500) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 74) {
          if (features[0] <= 40) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[0] <= 51) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 12) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 125000) {
        if (features[1] <= 75000) {
          if (features[1] <= 55000) {
            if (features[1] <= 45000) {
              if (features[0] <= 51) {
                if (features[0] <= 31) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 51) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            if (features[0] <= 28) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 85000) {
            if (features[0] <= 31) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 28) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 36
int spinlock_predict_tree_36(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 45000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 55000) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[1] <= 45000) {
        if (features[1] <= 12500) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 51) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 37
int spinlock_predict_tree_37(int features[]) {
  if (features[1] <= 125000) {
    if (features[1] <= 25000) {
      if (features[0] <= 51) {
        if (features[0] <= 28) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 40) {
            return 2; // FDS_TCLOCK
          } else {
            return 1; // FDS_QSPINLOCK
          }
        }
      } else {
        if (features[1] <= 7500) {
          if (features[0] <= 80) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[1] <= 45000) {
          if (features[0] <= 28) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 45) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[1] <= 35000) {
                if (features[0] <= 80) {
                  if (features[0] <= 63) {
                    return 3; // FDS_TDLOCK
                  } else {
                    return 2; // FDS_TCLOCK
                  }
                } else {
                  return 3; // FDS_TDLOCK
                }
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[0] <= 12) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 40) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 28) {
          if (features[0] <= 12) {
            if (features[1] <= 85000) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 6) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 5) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 12) {
        if (features[1] <= 175000) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 38
int spinlock_predict_tree_38(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 45000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[0] <= 12) {
          if (features[1] <= 65000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 120000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[1] <= 175000) {
                return 0; // FDS_AQS
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[0] <= 74) {
        if (features[1] <= 7500) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 63) {
        return 3; // FDS_TDLOCK
      } else {
        if (features[0] <= 74) {
          if (features[1] <= 45000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 39
int spinlock_predict_tree_39(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 125000) {
      if (features[1] <= 95000) {
        if (features[0] <= 19) {
          if (features[0] <= 12) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 45000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 40000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 12) {
        if (features[0] <= 2) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 175000) {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 0; // FDS_AQS
            }
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[1] <= 55000) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 86) {
        if (features[1] <= 25000) {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 74) {
              if (features[1] <= 7500) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 35000) {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              if (features[0] <= 74) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            if (features[1] <= 45000) {
              if (features[0] <= 51) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 40
int spinlock_predict_tree_40(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 45000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 125000) {
        if (features[1] <= 55000) {
          if (features[0] <= 9) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 75000) {
            if (features[0] <= 15) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 95000) {
              if (features[0] <= 6) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 6) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          }
        }
      } else {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 175000) {
            if (features[0] <= 12) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 6) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[0] <= 45) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      return 3; // FDS_TDLOCK
    }
  }
}

// Tree 41
int spinlock_predict_tree_41(int features[]) {
  if (features[1] <= 25000) {
    if (features[0] <= 74) {
      if (features[0] <= 51) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 63) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      }
    } else {
      return 2; // FDS_TCLOCK
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 12) {
        if (features[1] <= 75000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 45000) {
          if (features[1] <= 35000) {
            if (features[0] <= 74) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 28) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 6) {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 2) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 42
int spinlock_predict_tree_42(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 12) {
      if (features[1] <= 175000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 4) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 19) {
        if (features[1] <= 55000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 40000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 135000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[0] <= 51) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 55000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 51) {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 35000) {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              if (features[0] <= 74) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 43
int spinlock_predict_tree_43(int features[]) {
  if (features[1] <= 25000) {
    if (features[0] <= 63) {
      if (features[0] <= 51) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 12500) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 74) {
        if (features[1] <= 7500) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 6) {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      } else {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 125000) {
                return 2; // FDS_TCLOCK
              } else {
                if (features[1] <= 175000) {
                  return 0; // FDS_AQS
                } else {
                  return 3; // FDS_TDLOCK
                }
              }
            }
          } else {
            if (features[0] <= 19) {
              if (features[1] <= 120000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[1] <= 55000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 44
int spinlock_predict_tree_44(int features[]) {
  if (features[1] <= 25000) {
    if (features[0] <= 51) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 80) {
        if (features[1] <= 7500) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[1] <= 55000) {
        if (features[0] <= 51) {
          if (features[0] <= 19) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 74) {
            if (features[1] <= 35000) {
              if (features[0] <= 63) {
                return 3; // FDS_TDLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 65000) {
          if (features[0] <= 28) {
            if (features[0] <= 15) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 13) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 28) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 6) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 150000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[0] <= 28) {
          if (features[0] <= 19) {
            if (features[1] <= 125000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[1] <= 175000) {
                if (features[0] <= 12) {
                  return 0; // FDS_AQS
                } else {
                  return 3; // FDS_TDLOCK
                }
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 45
int spinlock_predict_tree_45(int features[]) {
  if (features[0] <= 19) {
    if (features[1] <= 125000) {
      if (features[1] <= 65000) {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 10) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 3) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          return 0; // FDS_AQS
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 15000) {
        if (features[1] <= 7500) {
          if (features[0] <= 68) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 57) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 75000) {
          if (features[0] <= 28) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 55000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 51) {
          if (features[1] <= 50000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 46
int spinlock_predict_tree_46(int features[]) {
  if (features[1] <= 25000) {
    if (features[0] <= 28) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 7500) {
        if (features[0] <= 68) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[0] <= 12) {
      if (features[1] <= 75000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          if (features[1] <= 175000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 3) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 0; // FDS_AQS
            }
          }
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 45) {
          if (features[1] <= 40000) {
            if (features[0] <= 28) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[1] <= 35000) {
              if (features[0] <= 80) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 125000) {
          if (features[1] <= 95000) {
            if (features[0] <= 28) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 28) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 47
int spinlock_predict_tree_47(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 45000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 65000) {
            if (features[1] <= 55000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[0] <= 12) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 175000) {
          if (features[0] <= 10) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 9) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 7500) {
        if (features[0] <= 68) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 51) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 48
int spinlock_predict_tree_48(int features[]) {
  if (features[1] <= 25000) {
    if (features[0] <= 80) {
      if (features[1] <= 7500) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 15000) {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      }
    } else {
      return 2; // FDS_TCLOCK
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 12) {
        if (features[0] <= 6) {
          if (features[1] <= 175000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 2) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 0; // FDS_AQS
            }
          }
        } else {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 150000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 19) {
          if (features[1] <= 50000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 125000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 145000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 60000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 35000) {
          if (features[0] <= 74) {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 49
int spinlock_predict_tree_49(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 125000) {
      if (features[1] <= 45000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 55000) {
          if (features[0] <= 12) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 12) {
              if (features[1] <= 75000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 10) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 6) {
          if (features[0] <= 2) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 51) {
      if (features[1] <= 55000) {
        if (features[0] <= 40) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 22500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 63) {
        return 3; // FDS_TDLOCK
      } else {
        if (features[0] <= 74) {
          if (features[1] <= 35000) {
            if (features[1] <= 7500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 50
int spinlock_predict_tree_50(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 12) {
      if (features[1] <= 175000) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 6) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 19) {
        if (features[1] <= 35000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 50000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[0] <= 51) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 74) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[1] <= 45000) {
          return 3; // FDS_TDLOCK
        } else {
          if (features[0] <= 45) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 51
int spinlock_predict_tree_51(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 12) {
      if (features[1] <= 125000) {
        if (features[1] <= 85000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 95000) {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_QSPINLOCK
          }
        }
      } else {
        if (features[1] <= 175000) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[0] <= 19) {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 40000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 51) {
      if (features[1] <= 55000) {
        if (features[1] <= 25000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 63) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 74) {
            if (features[1] <= 12500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 35000) {
          if (features[0] <= 74) {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 52
int spinlock_predict_tree_52(int features[]) {
  if (features[0] <= 19) {
    if (features[0] <= 6) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 55000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 12) {
          if (features[1] <= 80000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[1] <= 40000) {
        if (features[1] <= 25000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 28) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 28) {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[1] <= 55000) {
          if (features[1] <= 25000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 63) {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 35000) {
            if (features[1] <= 20000) {
              if (features[1] <= 7500) {
                if (features[0] <= 74) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 74) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 53
int spinlock_predict_tree_53(int features[]) {
  if (features[0] <= 12) {
    if (features[0] <= 6) {
      if (features[1] <= 175000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[1] <= 85000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 150000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 7500) {
          if (features[0] <= 74) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 28) {
          if (features[1] <= 45000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 55000) {
            if (features[0] <= 51) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[0] <= 74) {
                if (features[0] <= 63) {
                  return 3; // FDS_TDLOCK
                } else {
                  if (features[1] <= 35000) {
                    return 2; // FDS_TCLOCK
                  } else {
                    return 3; // FDS_TDLOCK
                  }
                }
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 54
int spinlock_predict_tree_54(int features[]) {
  if (features[1] <= 25000) {
    if (features[0] <= 74) {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 63) {
          if (features[0] <= 51) {
            if (features[0] <= 40) {
              if (features[1] <= 15000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_QSPINLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      }
    } else {
      return 2; // FDS_TCLOCK
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 75000) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[0] <= 51) {
          if (features[0] <= 40) {
            if (features[1] <= 60000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 60000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 8) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 5) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 55
int spinlock_predict_tree_55(int features[]) {
  if (features[1] <= 45000) {
    if (features[0] <= 45) {
      if (features[1] <= 15000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 25000) {
          if (features[0] <= 28) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[1] <= 7500) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[0] <= 12) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 80000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 55000) {
          if (features[0] <= 51) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 21) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 10) {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 56
int spinlock_predict_tree_56(int features[]) {
  if (features[1] <= 25000) {
    if (features[1] <= 7500) {
      if (features[0] <= 74) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 15000) {
          if (features[0] <= 57) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    }
  } else {
    if (features[1] <= 95000) {
      if (features[0] <= 19) {
        if (features[0] <= 12) {
          if (features[1] <= 85000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 50000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 28) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 35000) {
            if (features[0] <= 74) {
              if (features[0] <= 63) {
                if (features[0] <= 45) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 40) {
              if (features[1] <= 55000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[0] <= 10) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 150000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 57
int spinlock_predict_tree_57(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 125000) {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 12) {
          if (features[1] <= 65000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 40000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 3) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 175000) {
          if (features[0] <= 12) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 6) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[1] <= 60000) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 51) {
        return 3; // FDS_TDLOCK
      } else {
        if (features[0] <= 74) {
          if (features[1] <= 35000) {
            if (features[0] <= 63) {
              if (features[1] <= 12500) {
                return 1; // FDS_QSPINLOCK
              } else {
                if (features[1] <= 25000) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 58
int spinlock_predict_tree_58(int features[]) {
  if (features[1] <= 45000) {
    if (features[0] <= 51) {
      if (features[1] <= 15000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 25000) {
          if (features[0] <= 28) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 63) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 74) {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[0] <= 12) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 70000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 40) {
          if (features[1] <= 60000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 21) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 12) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 59
int spinlock_predict_tree_59(int features[]) {
  if (features[1] <= 85000) {
    if (features[0] <= 51) {
      if (features[1] <= 55000) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 25000) {
            if (features[0] <= 25) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 28) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[1] <= 65000) {
          if (features[0] <= 12) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 75000) {
              if (features[0] <= 28) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 74) {
          if (features[0] <= 63) {
            if (features[1] <= 7500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 12500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[1] <= 95000) {
      if (features[0] <= 28) {
        return 2; // FDS_TCLOCK
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 12) {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 125000) {
            if (features[0] <= 25) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 60
int spinlock_predict_tree_60(int features[]) {
  if (features[0] <= 19) {
    if (features[0] <= 6) {
      if (features[1] <= 175000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[1] <= 75000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 12) {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[1] <= 40000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 150000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 51) {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 51) {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 35000) {
            if (features[0] <= 74) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

// Tree 61
int spinlock_predict_tree_61(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 12) {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 75000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 120000) {
            return 2; // FDS_TCLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      }
    } else {
      if (features[0] <= 19) {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 115000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    }
  } else {
    if (features[0] <= 51) {
      if (features[0] <= 40) {
        if (features[1] <= 55000) {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 60000) {
          if (features[1] <= 22500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 74) {
          if (features[1] <= 12500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 35000) {
          if (features[0] <= 80) {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 62
int spinlock_predict_tree_62(int features[]) {
  if (features[0] <= 12) {
    if (features[1] <= 125000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 4) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      } else {
        return 0; // FDS_AQS
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 7500) {
        if (features[0] <= 80) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 15000) {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 28) {
        if (features[0] <= 19) {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 45000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 45000) {
          if (features[1] <= 35000) {
            if (features[0] <= 74) {
              if (features[0] <= 45) {
                return 2; // FDS_TCLOCK
              } else {
                if (features[0] <= 63) {
                  return 3; // FDS_TDLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[0] <= 51) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 63
int spinlock_predict_tree_63(int features[]) {
  if (features[1] <= 25000) {
    if (features[1] <= 15000) {
      if (features[0] <= 57) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 21) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[0] <= 40) {
      if (features[0] <= 6) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 145000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[1] <= 125000) {
          if (features[1] <= 75000) {
            if (features[1] <= 40000) {
              if (features[0] <= 28) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 55000) {
                return 2; // FDS_TCLOCK
              } else {
                if (features[1] <= 65000) {
                  if (features[0] <= 12) {
                    return 1; // FDS_QSPINLOCK
                  } else {
                    return 2; // FDS_TCLOCK
                  }
                } else {
                  if (features[0] <= 12) {
                    return 1; // FDS_QSPINLOCK
                  } else {
                    return 2; // FDS_TCLOCK
                  }
                }
              }
            }
          } else {
            if (features[1] <= 95000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[0] <= 28) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 45000) {
        if (features[1] <= 35000) {
          if (features[0] <= 74) {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 51) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 64
int spinlock_predict_tree_64(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 175000) {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 55000) {
          if (features[1] <= 45000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 120000) {
                return 2; // FDS_TCLOCK
              } else {
                return 0; // FDS_AQS
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 6) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[0] <= 63) {
      if (features[1] <= 15000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 51) {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 20000) {
        return 2; // FDS_TCLOCK
      } else {
        if (features[1] <= 35000) {
          if (features[0] <= 74) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 65
int spinlock_predict_tree_65(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 12) {
      if (features[0] <= 3) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          if (features[1] <= 175000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          if (features[1] <= 80000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 19) {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 40000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 130000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 7500) {
        if (features[0] <= 74) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 15000) {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 35000) {
        if (features[0] <= 74) {
          if (features[0] <= 63) {
            if (features[0] <= 45) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 55000) {
          if (features[1] <= 45000) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[0] <= 45) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 66
int spinlock_predict_tree_66(int features[]) {
  if (features[1] <= 95000) {
    if (features[1] <= 15000) {
      if (features[1] <= 7500) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 51) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 28) {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 75000) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 40) {
            if (features[1] <= 60000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 35000) {
              if (features[0] <= 63) {
                return 3; // FDS_TDLOCK
              } else {
                if (features[0] <= 74) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 34) {
        if (features[0] <= 12) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 12) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 10) {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 67
int spinlock_predict_tree_67(int features[]) {
  if (features[0] <= 12) {
    if (features[0] <= 6) {
      if (features[0] <= 3) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[1] <= 70000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 120000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 175000) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[0] <= 63) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 25000) {
            if (features[0] <= 25) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 28) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 45) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 28) {
          if (features[0] <= 19) {
            if (features[1] <= 45000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 55000) {
            if (features[1] <= 45000) {
              if (features[0] <= 51) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 45) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 68
int spinlock_predict_tree_68(int features[]) {
  if (features[1] <= 15000) {
    if (features[1] <= 7500) {
      if (features[0] <= 80) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 51) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[0] <= 12) {
      if (features[1] <= 125000) {
        if (features[1] <= 75000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 90000) {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[0] <= 6) {
          if (features[1] <= 175000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          if (features[1] <= 175000) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 25000) {
        return 2; // FDS_TCLOCK
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 28) {
            if (features[0] <= 19) {
              if (features[1] <= 45000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 51) {
              if (features[1] <= 55000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 69
int spinlock_predict_tree_69(int features[]) {
  if (features[1] <= 45000) {
    if (features[1] <= 7500) {
      if (features[0] <= 74) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 45) {
        if (features[1] <= 25000) {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 28) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 74) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[1] <= 75000) {
          if (features[1] <= 55000) {
            if (features[0] <= 9) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 15) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 85000) {
            if (features[0] <= 5) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 95000) {
              if (features[0] <= 9) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              if (features[0] <= 12) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          }
        }
      } else {
        if (features[0] <= 40) {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 175000) {
        if (features[0] <= 8) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 13) {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 70
int spinlock_predict_tree_70(int features[]) {
  if (features[0] <= 40) {
    if (features[1] <= 45000) {
      if (features[1] <= 25000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 35000) {
          if (features[0] <= 28) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      }
    } else {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 28) {
            if (features[1] <= 65000) {
              if (features[0] <= 12) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 55000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 15) {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 51) {
      return 3; // FDS_TDLOCK
    } else {
      if (features[1] <= 35000) {
        if (features[0] <= 63) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 25000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[0] <= 74) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 71
int spinlock_predict_tree_71(int features[]) {
  if (features[1] <= 25000) {
    if (features[1] <= 15000) {
      if (features[1] <= 7500) {
        if (features[0] <= 68) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 51) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[0] <= 12) {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 19) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 45000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 6) {
        if (features[0] <= 2) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      } else {
        if (features[1] <= 175000) {
          if (features[0] <= 12) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 72
int spinlock_predict_tree_72(int features[]) {
  if (features[0] <= 19) {
    if (features[1] <= 125000) {
      if (features[0] <= 12) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 50000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 6) {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[1] <= 175000) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[0] <= 74) {
        if (features[0] <= 51) {
          if (features[0] <= 40) {
            if (features[1] <= 15000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_QSPINLOCK
          }
        } else {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 20000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 28) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 73
int spinlock_predict_tree_73(int features[]) {
  if (features[1] <= 25000) {
    if (features[1] <= 7500) {
      if (features[0] <= 74) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 15000) {
        if (features[0] <= 51) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 25) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[1] <= 55000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 95000) {
          if (features[0] <= 12) {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 75000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 125000) {
            if (features[0] <= 9) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 6) {
              if (features[1] <= 175000) {
                return 1; // FDS_QSPINLOCK
              } else {
                if (features[0] <= 3) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  return 0; // FDS_AQS
                }
              }
            } else {
              if (features[1] <= 175000) {
                if (features[0] <= 12) {
                  return 0; // FDS_AQS
                } else {
                  return 3; // FDS_TDLOCK
                }
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      return 3; // FDS_TDLOCK
    }
  }
}

// Tree 74
int spinlock_predict_tree_74(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 6) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 40000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 12) {
            if (features[1] <= 70000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 175000) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[1] <= 7500) {
        if (features[0] <= 80) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 40) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 25000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[0] <= 80) {
                if (features[0] <= 63) {
                  return 3; // FDS_TDLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[0] <= 40) {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 50000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 75
int spinlock_predict_tree_75(int features[]) {
  if (features[0] <= 12) {
    if (features[0] <= 6) {
      if (features[0] <= 3) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[1] <= 65000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 175000) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 7500) {
        if (features[0] <= 63) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 15000) {
          if (features[0] <= 45) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 28) {
        if (features[0] <= 19) {
          if (features[1] <= 55000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 125000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 55000) {
          if (features[1] <= 35000) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[1] <= 45000) {
              if (features[0] <= 57) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 45) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 76
int spinlock_predict_tree_76(int features[]) {
  if (features[1] <= 125000) {
    if (features[1] <= 25000) {
      if (features[1] <= 7500) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 15000) {
          if (features[0] <= 46) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 28) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[1] <= 35000) {
        if (features[0] <= 40) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 95000) {
          if (features[1] <= 85000) {
            if (features[0] <= 12) {
              if (features[0] <= 6) {
                return 1; // FDS_QSPINLOCK
              } else {
                if (features[1] <= 75000) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              }
            } else {
              if (features[0] <= 28) {
                return 2; // FDS_TCLOCK
              } else {
                if (features[0] <= 51) {
                  if (features[0] <= 40) {
                    if (features[1] <= 60000) {
                      return 2; // FDS_TCLOCK
                    } else {
                      return 3; // FDS_TDLOCK
                    }
                  } else {
                    if (features[1] <= 50000) {
                      return 2; // FDS_TCLOCK
                    } else {
                      return 3; // FDS_TDLOCK
                    }
                  }
                } else {
                  return 3; // FDS_TDLOCK
                }
              }
            }
          } else {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 25) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          if (features[0] <= 28) {
            if (features[0] <= 4) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 6) {
      if (features[0] <= 3) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[0] <= 12) {
        if (features[1] <= 175000) {
          return 0; // FDS_AQS
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 77
int spinlock_predict_tree_77(int features[]) {
  if (features[0] <= 12) {
    if (features[0] <= 6) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 75000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 120000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 175000) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 19) {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 40000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 63) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 74) {
            if (features[1] <= 7500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 40) {
          if (features[1] <= 60000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[1] <= 35000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    }
  }
}

// Tree 78
int spinlock_predict_tree_78(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 12) {
      if (features[0] <= 3) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 6) {
          if (features[1] <= 150000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 115000) {
              return 2; // FDS_TCLOCK
            } else {
              return 0; // FDS_AQS
            }
          }
        }
      }
    } else {
      if (features[1] <= 45000) {
        if (features[0] <= 28) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 28) {
          if (features[0] <= 19) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 150000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 55000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 15000) {
        if (features[0] <= 74) {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 63) {
              if (features[1] <= 7500) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 1; // FDS_QSPINLOCK
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 45000) {
        if (features[0] <= 51) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 79
int spinlock_predict_tree_79(int features[]) {
  if (features[1] <= 25000) {
    if (features[0] <= 28) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 15000) {
        if (features[1] <= 7500) {
          if (features[0] <= 74) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 57) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 19) {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 145000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            if (features[1] <= 50000) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 125000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 40) {
        if (features[1] <= 60000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 74) {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[1] <= 40000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 80
int spinlock_predict_tree_80(int features[]) {
  if (features[1] <= 75000) {
    if (features[0] <= 19) {
      if (features[0] <= 12) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 50000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 51) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 35000) {
            if (features[1] <= 25000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[0] <= 28) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 25000) {
          if (features[0] <= 80) {
            if (features[0] <= 63) {
              if (features[1] <= 7500) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              if (features[1] <= 12500) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 15) {
        if (features[1] <= 175000) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        } else {
          return 1; // FDS_QSPINLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 81
int spinlock_predict_tree_81(int features[]) {
  if (features[0] <= 6) {
    return 1; // FDS_QSPINLOCK
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 19) {
        if (features[0] <= 12) {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 145000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[1] <= 125000) {
            if (features[1] <= 45000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 40000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 74) {
        if (features[0] <= 40) {
          if (features[1] <= 45000) {
            if (features[1] <= 17500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 63) {
            if (features[1] <= 50000) {
              if (features[1] <= 30000) {
                return 2; // FDS_TCLOCK
              } else {
                if (features[0] <= 51) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 40000) {
              if (features[1] <= 7500) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 86) {
          if (features[1] <= 17500) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 82
int spinlock_predict_tree_82(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 6) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 45000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 150000) {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[0] <= 51) {
      if (features[0] <= 40) {
        if (features[1] <= 45000) {
          if (features[1] <= 20000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 63) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 7500) {
            if (features[0] <= 74) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 74) {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[1] <= 35000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 83
int spinlock_predict_tree_83(int features[]) {
  if (features[1] <= 45000) {
    if (features[1] <= 25000) {
      if (features[1] <= 15000) {
        if (features[1] <= 7500) {
          if (features[0] <= 69) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 28) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 31) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 35000) {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[0] <= 80) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 51) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[0] <= 28) {
        if (features[1] <= 55000) {
          if (features[0] <= 10) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[1] <= 65000) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 10) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        }
      } else {
        if (features[0] <= 40) {
          if (features[1] <= 60000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 28) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 5) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 84
int spinlock_predict_tree_84(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 45000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 125000) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 70000) {
            if (features[1] <= 55000) {
              return 2; // FDS_TCLOCK
            } else {
              if (features[0] <= 15) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 175000) {
          if (features[0] <= 12) {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 0; // FDS_AQS
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 15000) {
        if (features[1] <= 7500) {
          if (features[0] <= 68) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 51) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 35000) {
            if (features[0] <= 74) {
              if (features[0] <= 63) {
                return 3; // FDS_TDLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 85
int spinlock_predict_tree_85(int features[]) {
  if (features[1] <= 25000) {
    if (features[1] <= 15000) {
      if (features[1] <= 7500) {
        if (features[0] <= 74) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 57) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 42) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[1] <= 65000) {
        if (features[1] <= 45000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 55000) {
            if (features[0] <= 15) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 1; // FDS_QSPINLOCK
          }
        }
      } else {
        if (features[1] <= 125000) {
          if (features[1] <= 85000) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[0] <= 6) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 3) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 6) {
              if (features[1] <= 175000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 0; // FDS_AQS
              }
            } else {
              return 0; // FDS_AQS
            }
          }
        }
      }
    } else {
      if (features[1] <= 35000) {
        if (features[0] <= 45) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 86
int spinlock_predict_tree_86(int features[]) {
  if (features[1] <= 55000) {
    if (features[0] <= 51) {
      if (features[0] <= 19) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 35000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[1] <= 25000) {
        return 2; // FDS_TCLOCK
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[1] <= 75000) {
      if (features[0] <= 28) {
        if (features[0] <= 10) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[0] <= 6) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 150000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[0] <= 19) {
          if (features[0] <= 12) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 150000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 87
int spinlock_predict_tree_87(int features[]) {
  if (features[1] <= 25000) {
    if (features[1] <= 7500) {
      if (features[0] <= 68) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 45) {
        if (features[0] <= 28) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    }
  } else {
    if (features[1] <= 95000) {
      if (features[1] <= 35000) {
        if (features[0] <= 25) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 80) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 28) {
          if (features[1] <= 55000) {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 45000) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            if (features[0] <= 12) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 55000) {
            if (features[1] <= 45000) {
              if (features[0] <= 63) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            } else {
              if (features[0] <= 51) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 12) {
        if (features[1] <= 125000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 175000) {
            if (features[0] <= 4) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 0; // FDS_AQS
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[0] <= 28) {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 88
int spinlock_predict_tree_88(int features[]) {
  if (features[1] <= 35000) {
    if (features[0] <= 28) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 25000) {
        if (features[0] <= 63) {
          if (features[1] <= 12500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[0] <= 45) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[0] <= 74) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    }
  } else {
    if (features[1] <= 175000) {
      if (features[0] <= 12) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 28) {
          if (features[1] <= 45000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 120000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 51) {
            if (features[0] <= 40) {
              return 3; // FDS_TDLOCK
            } else {
              if (features[1] <= 50000) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 6) {
        return 0; // FDS_AQS
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 89
int spinlock_predict_tree_89(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 45000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 12) {
        if (features[0] <= 6) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 75000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 150000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[0] <= 19) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 63) {
      if (features[0] <= 40) {
        if (features[1] <= 37500) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 50000) {
          if (features[0] <= 51) {
            if (features[1] <= 22500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            if (features[1] <= 30000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 86) {
        if (features[1] <= 20000) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 90
int spinlock_predict_tree_90(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 75000) {
      if (features[0] <= 12) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 50000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    } else {
      if (features[0] <= 6) {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 2) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 0; // FDS_AQS
          }
        }
      } else {
        if (features[1] <= 125000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 175000) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 35000) {
      if (features[1] <= 20000) {
        if (features[0] <= 51) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 7500) {
            if (features[0] <= 74) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[0] <= 45) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 51) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 91
int spinlock_predict_tree_91(int features[]) {
  if (features[0] <= 40) {
    if (features[0] <= 6) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 28) {
        if (features[1] <= 40000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 125000) {
                return 2; // FDS_TCLOCK
              } else {
                return 0; // FDS_AQS
              }
            }
          } else {
            if (features[1] <= 150000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      } else {
        if (features[1] <= 20000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 60000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 63) {
      if (features[0] <= 51) {
        if (features[1] <= 50000) {
          if (features[1] <= 22500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 20000) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 86) {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 25000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 92
int spinlock_predict_tree_92(int features[]) {
  if (features[0] <= 6) {
    return 1; // FDS_QSPINLOCK
  } else {
    if (features[1] <= 45000) {
      if (features[1] <= 25000) {
        if (features[0] <= 51) {
          if (features[1] <= 15000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 28) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[1] <= 7500) {
            if (features[0] <= 80) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 35000) {
          if (features[0] <= 28) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 45) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          if (features[0] <= 27) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[0] <= 63) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 57) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 28) {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 93
int spinlock_predict_tree_93(int features[]) {
  if (features[1] <= 25000) {
    if (features[1] <= 7500) {
      if (features[0] <= 74) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 15000) {
          if (features[0] <= 51) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[1] <= 95000) {
        if (features[0] <= 12) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 19) {
            if (features[1] <= 50000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        if (features[1] <= 125000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 175000) {
            if (features[0] <= 4) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 12) {
                return 0; // FDS_AQS
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            if (features[0] <= 2) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 6) {
                return 0; // FDS_AQS
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          }
        }
      }
    } else {
      if (features[1] <= 55000) {
        if (features[0] <= 45) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 35000) {
            if (features[0] <= 63) {
              return 3; // FDS_TDLOCK
            } else {
              if (features[0] <= 74) {
                return 2; // FDS_TCLOCK
              } else {
                return 3; // FDS_TDLOCK
              }
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 94
int spinlock_predict_tree_94(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 45000) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 6) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 12) {
          if (features[1] <= 70000) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 125000) {
              return 2; // FDS_TCLOCK
            } else {
              return 0; // FDS_AQS
            }
          }
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 51) {
      if (features[0] <= 40) {
        if (features[1] <= 55000) {
          if (features[1] <= 12500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[1] <= 35000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 74) {
        if (features[0] <= 63) {
          if (features[1] <= 35000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[1] <= 35000) {
            if (features[1] <= 7500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 30000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 95
int spinlock_predict_tree_95(int features[]) {
  if (features[0] <= 28) {
    if (features[1] <= 175000) {
      if (features[1] <= 45000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 85000) {
                if (features[0] <= 6) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  return 2; // FDS_TCLOCK
                }
              } else {
                if (features[1] <= 95000) {
                  return 1; // FDS_QSPINLOCK
                } else {
                  if (features[0] <= 6) {
                    return 1; // FDS_QSPINLOCK
                  } else {
                    return 2; // FDS_TCLOCK
                  }
                }
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 10) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 6) {
        if (features[0] <= 3) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[0] <= 51) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[1] <= 45000) {
        if (features[1] <= 35000) {
          if (features[0] <= 63) {
            return 3; // FDS_TDLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 51) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 96
int spinlock_predict_tree_96(int features[]) {
  if (features[1] <= 35000) {
    if (features[1] <= 25000) {
      if (features[0] <= 28) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[0] <= 63) {
          if (features[0] <= 40) {
            return 2; // FDS_TCLOCK
          } else {
            if (features[1] <= 12500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          if (features[0] <= 74) {
            if (features[1] <= 7500) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      }
    } else {
      if (features[0] <= 36) {
        return 1; // FDS_QSPINLOCK
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  } else {
    if (features[1] <= 125000) {
      if (features[0] <= 28) {
        if (features[0] <= 12) {
          if (features[0] <= 6) {
            return 1; // FDS_QSPINLOCK
          } else {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          }
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 55000) {
          if (features[1] <= 45000) {
            return 3; // FDS_TDLOCK
          } else {
            if (features[0] <= 57) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    } else {
      if (features[0] <= 5) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 175000) {
          if (features[0] <= 21) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 97
int spinlock_predict_tree_97(int features[]) {
  if (features[0] <= 12) {
    if (features[0] <= 6) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[1] <= 75000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 120000) {
          return 2; // FDS_TCLOCK
        } else {
          if (features[1] <= 175000) {
            return 0; // FDS_AQS
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[0] <= 28) {
      if (features[0] <= 19) {
        if (features[1] <= 40000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 125000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      } else {
        if (features[1] <= 30000) {
          return 1; // FDS_QSPINLOCK
        } else {
          if (features[1] <= 150000) {
            return 2; // FDS_TCLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    } else {
      if (features[0] <= 63) {
        if (features[1] <= 55000) {
          if (features[1] <= 7500) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        if (features[0] <= 86) {
          if (features[0] <= 74) {
            if (features[1] <= 35000) {
              if (features[1] <= 7500) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            if (features[1] <= 30000) {
              return 2; // FDS_TCLOCK
            } else {
              return 3; // FDS_TDLOCK
            }
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  }
}

// Tree 98
int spinlock_predict_tree_98(int features[]) {
  if (features[0] <= 28) {
    if (features[0] <= 6) {
      if (features[0] <= 3) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 175000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 0; // FDS_AQS
        }
      }
    } else {
      if (features[1] <= 45000) {
        return 1; // FDS_QSPINLOCK
      } else {
        if (features[1] <= 125000) {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 2; // FDS_TCLOCK
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 12) {
            if (features[1] <= 175000) {
              return 0; // FDS_AQS
            } else {
              return 3; // FDS_TDLOCK
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  } else {
    if (features[1] <= 25000) {
      if (features[1] <= 15000) {
        if (features[1] <= 7500) {
          if (features[0] <= 68) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 63) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 2; // FDS_TCLOCK
      }
    } else {
      if (features[0] <= 51) {
        if (features[1] <= 50000) {
          return 2; // FDS_TCLOCK
        } else {
          return 3; // FDS_TDLOCK
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    }
  }
}

// Tree 99
int spinlock_predict_tree_99(int features[]) {
  if (features[1] <= 55000) {
    if (features[0] <= 28) {
      return 1; // FDS_QSPINLOCK
    } else {
      if (features[0] <= 51) {
        if (features[1] <= 15000) {
          return 1; // FDS_QSPINLOCK
        } else {
          return 2; // FDS_TCLOCK
        }
      } else {
        if (features[1] <= 35000) {
          if (features[0] <= 86) {
            if (features[1] <= 12500) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[1] <= 25000) {
                return 2; // FDS_TCLOCK
              } else {
                if (features[0] <= 74) {
                  return 2; // FDS_TCLOCK
                } else {
                  return 3; // FDS_TDLOCK
                }
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          return 3; // FDS_TDLOCK
        }
      }
    }
  } else {
    if (features[1] <= 95000) {
      if (features[0] <= 28) {
        if (features[1] <= 85000) {
          if (features[0] <= 12) {
            if (features[1] <= 75000) {
              return 1; // FDS_QSPINLOCK
            } else {
              if (features[0] <= 6) {
                return 1; // FDS_QSPINLOCK
              } else {
                return 2; // FDS_TCLOCK
              }
            }
          } else {
            return 2; // FDS_TCLOCK
          }
        } else {
          if (features[0] <= 10) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 2; // FDS_TCLOCK
          }
        }
      } else {
        return 3; // FDS_TDLOCK
      }
    } else {
      if (features[1] <= 125000) {
        return 3; // FDS_TDLOCK
      } else {
        if (features[1] <= 175000) {
          if (features[0] <= 30) {
            return 1; // FDS_QSPINLOCK
          } else {
            return 3; // FDS_TDLOCK
          }
        } else {
          if (features[0] <= 6) {
            if (features[0] <= 3) {
              return 1; // FDS_QSPINLOCK
            } else {
              return 0; // FDS_AQS
            }
          } else {
            return 3; // FDS_TDLOCK
          }
        }
      }
    }
  }
}

int predict_spinlock_random_forest(int features[]) {
  int predictions[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  predictions[0] = spinlock_predict_tree_0(features);
  predictions[1] = spinlock_predict_tree_1(features);
  predictions[2] = spinlock_predict_tree_2(features);
  predictions[3] = spinlock_predict_tree_3(features);
  predictions[4] = spinlock_predict_tree_4(features);
  predictions[5] = spinlock_predict_tree_5(features);
  predictions[6] = spinlock_predict_tree_6(features);
  predictions[7] = spinlock_predict_tree_7(features);
  predictions[8] = spinlock_predict_tree_8(features);
  predictions[9] = spinlock_predict_tree_9(features);
  predictions[10] = spinlock_predict_tree_10(features);
  predictions[11] = spinlock_predict_tree_11(features);
  predictions[12] = spinlock_predict_tree_12(features);
  predictions[13] = spinlock_predict_tree_13(features);
  predictions[14] = spinlock_predict_tree_14(features);
  predictions[15] = spinlock_predict_tree_15(features);
  predictions[16] = spinlock_predict_tree_16(features);
  predictions[17] = spinlock_predict_tree_17(features);
  predictions[18] = spinlock_predict_tree_18(features);
  predictions[19] = spinlock_predict_tree_19(features);
  predictions[20] = spinlock_predict_tree_20(features);
  predictions[21] = spinlock_predict_tree_21(features);
  predictions[22] = spinlock_predict_tree_22(features);
  predictions[23] = spinlock_predict_tree_23(features);
  predictions[24] = spinlock_predict_tree_24(features);
  predictions[25] = spinlock_predict_tree_25(features);
  predictions[26] = spinlock_predict_tree_26(features);
  predictions[27] = spinlock_predict_tree_27(features);
  predictions[28] = spinlock_predict_tree_28(features);
  predictions[29] = spinlock_predict_tree_29(features);
  predictions[30] = spinlock_predict_tree_30(features);
  predictions[31] = spinlock_predict_tree_31(features);
  predictions[32] = spinlock_predict_tree_32(features);
  predictions[33] = spinlock_predict_tree_33(features);
  predictions[34] = spinlock_predict_tree_34(features);
  predictions[35] = spinlock_predict_tree_35(features);
  predictions[36] = spinlock_predict_tree_36(features);
  predictions[37] = spinlock_predict_tree_37(features);
  predictions[38] = spinlock_predict_tree_38(features);
  predictions[39] = spinlock_predict_tree_39(features);
  predictions[40] = spinlock_predict_tree_40(features);
  predictions[41] = spinlock_predict_tree_41(features);
  predictions[42] = spinlock_predict_tree_42(features);
  predictions[43] = spinlock_predict_tree_43(features);
  predictions[44] = spinlock_predict_tree_44(features);
  predictions[45] = spinlock_predict_tree_45(features);
  predictions[46] = spinlock_predict_tree_46(features);
  predictions[47] = spinlock_predict_tree_47(features);
  predictions[48] = spinlock_predict_tree_48(features);
  predictions[49] = spinlock_predict_tree_49(features);
  predictions[50] = spinlock_predict_tree_50(features);
  predictions[51] = spinlock_predict_tree_51(features);
  predictions[52] = spinlock_predict_tree_52(features);
  predictions[53] = spinlock_predict_tree_53(features);
  predictions[54] = spinlock_predict_tree_54(features);
  predictions[55] = spinlock_predict_tree_55(features);
  predictions[56] = spinlock_predict_tree_56(features);
  predictions[57] = spinlock_predict_tree_57(features);
  predictions[58] = spinlock_predict_tree_58(features);
  predictions[59] = spinlock_predict_tree_59(features);
  predictions[60] = spinlock_predict_tree_60(features);
  predictions[61] = spinlock_predict_tree_61(features);
  predictions[62] = spinlock_predict_tree_62(features);
  predictions[63] = spinlock_predict_tree_63(features);
  predictions[64] = spinlock_predict_tree_64(features);
  predictions[65] = spinlock_predict_tree_65(features);
  predictions[66] = spinlock_predict_tree_66(features);
  predictions[67] = spinlock_predict_tree_67(features);
  predictions[68] = spinlock_predict_tree_68(features);
  predictions[69] = spinlock_predict_tree_69(features);
  predictions[70] = spinlock_predict_tree_70(features);
  predictions[71] = spinlock_predict_tree_71(features);
  predictions[72] = spinlock_predict_tree_72(features);
  predictions[73] = spinlock_predict_tree_73(features);
  predictions[74] = spinlock_predict_tree_74(features);
  predictions[75] = spinlock_predict_tree_75(features);
  predictions[76] = spinlock_predict_tree_76(features);
  predictions[77] = spinlock_predict_tree_77(features);
  predictions[78] = spinlock_predict_tree_78(features);
  predictions[79] = spinlock_predict_tree_79(features);
  predictions[80] = spinlock_predict_tree_80(features);
  predictions[81] = spinlock_predict_tree_81(features);
  predictions[82] = spinlock_predict_tree_82(features);
  predictions[83] = spinlock_predict_tree_83(features);
  predictions[84] = spinlock_predict_tree_84(features);
  predictions[85] = spinlock_predict_tree_85(features);
  predictions[86] = spinlock_predict_tree_86(features);
  predictions[87] = spinlock_predict_tree_87(features);
  predictions[88] = spinlock_predict_tree_88(features);
  predictions[89] = spinlock_predict_tree_89(features);
  predictions[90] = spinlock_predict_tree_90(features);
  predictions[91] = spinlock_predict_tree_91(features);
  predictions[92] = spinlock_predict_tree_92(features);
  predictions[93] = spinlock_predict_tree_93(features);
  predictions[94] = spinlock_predict_tree_94(features);
  predictions[95] = spinlock_predict_tree_95(features);
  predictions[96] = spinlock_predict_tree_96(features);
  predictions[97] = spinlock_predict_tree_97(features);
  predictions[98] = spinlock_predict_tree_98(features);
  predictions[99] = spinlock_predict_tree_99(features);
  int counts[4] = {0, 0, 0, 0};
  for (int i = 0; i < 100; i++) {
    counts[predictions[i]]++;
  }
  int max_count = 0;
  int max_index = 0;
  for (int i = 0; i < 4; i++) {
    if (counts[i] > max_count) {
      max_count = counts[i];
      max_index = i;
    }
  }
  return max_index;
}

