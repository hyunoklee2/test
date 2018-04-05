package com.schibsted.spain.parallaxlayerlayout.sample;

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.RadioGroup;
import com.schibsted.spain.parallaxlayerlayout.AnimatedTranslationUpdater;
import com.schibsted.spain.parallaxlayerlayout.ParallaxLayerLayout;
import com.schibsted.spain.parallaxlayerlayout.SensorTranslationUpdater;

public class MainActivity extends AppCompatActivity {

  private ParallaxLayerLayout parallaxLayout;
  private SensorTranslationUpdater translationUpdater;
  private float x_count = 0 ;
  private float y_count = 0 ;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    //setContentView(R.layout.activity_main_ovni);
    setContentView(R.layout.activity_main_squares);

    parallaxLayout = (ParallaxLayerLayout) findViewById(R.id.parallax);
    RadioGroup updaterGroup = (RadioGroup) findViewById(R.id.updater_group);

    translationUpdater = new SensorTranslationUpdater(this);

    //noinspection ConstantConditions
    updaterGroup.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
      @Override
      public void onCheckedChanged(RadioGroup group, int checkedId) {
        if (checkedId == R.id.updater_sensor_button) {
          parallaxLayout.setTranslationUpdater(translationUpdater);
        } else if (checkedId == R.id.updater_auto_button) {
          parallaxLayout.setTranslationUpdater(new AnimatedTranslationUpdater(0.5f));
        }
      }
    });

    updaterGroup.check(R.id.updater_sensor_button);

    // Resets orientation when clicked
    parallaxLayout.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View v) {
        translationUpdater.reset();
      }
    });

    Button left = (Button) findViewById(R.id.left) ;
    Button right = (Button) findViewById(R.id.right) ;
    Button up = (Button) findViewById(R.id.up) ;
    Button down = (Button) findViewById(R.id.down) ;



    left.setOnClickListener(new Button.OnClickListener() {
      @Override
      public void onClick(View view) {
        // TODO : click event
        x_count = x_count + (float) 0.1;
        if( Math.abs( x_count ) < 1) {
          parallaxLayout.updateTranslations(new float[]{(float) x_count, (float) y_count});
        }else{
          x_count = 1;
        }
      }
    });

    right.setOnClickListener(new Button.OnClickListener() {
      @Override
      public void onClick(View view) {
        // TODO : click event
        x_count = x_count - (float) 0.1;
        if( Math.abs( x_count ) < 1) {
          parallaxLayout.updateTranslations(new float[]{(float) x_count, (float) y_count});
        }else{
          x_count = -1;
        }
      }
    });

    up.setOnClickListener(new Button.OnClickListener() {
      @Override
      public void onClick(View view) {
        // TODO : click event
        y_count = y_count + (float) 0.1;
        if( Math.abs( y_count ) < 1) {
          parallaxLayout.updateTranslations(new float[]{(float) x_count, (float) y_count});
        }else{
          y_count = 1;
        }
      }
    });

    down.setOnClickListener(new Button.OnClickListener() {
      @Override
      public void onClick(View view) {
        // TODO : click event
          y_count = y_count - (float) 0.1;
        if( Math.abs( y_count ) < 1) {
          parallaxLayout.updateTranslations(new float[]{(float) x_count, (float) y_count});
        }else{
          y_count = -1;
        }
      }
    });

  }

  @Override
  protected void onResume() {
    super.onResume();
    translationUpdater.registerSensorManager();
  }

  @Override
  protected void onPause() {
    super.onPause();
    translationUpdater.unregisterSensorManager();
  }
}
